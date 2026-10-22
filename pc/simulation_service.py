"""Bounded local virtual sessions. No robot bridge, lease or hardware imports."""
from __future__ import annotations
import sys

import atexit
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time

PYTHON = Path(os.environ.get('XGODUCK_SIM_PYTHON') or sys.executable)


class SimulationSession:
    def __init__(self, root, model_id, expected_sha):
        self.id = secrets.token_urlsafe(24)
        self.model_id = model_id
        self.condition = threading.Condition()
        self.command_lock = threading.RLock()
        self.state = None
        self.acks = {}
        self.serial = 0
        self.errors = deque(maxlen=16)
        self.closed = False
        self.subscribers = 0
        self.idle_since = time.monotonic()
        self.idle_paused = False
        options = {'creationflags': subprocess.CREATE_NO_WINDOW} if hasattr(subprocess, 'CREATE_NO_WINDOW') else {}
        environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        self.process = subprocess.Popen([str(PYTHON), '-u', '-B', str(Path(root) / 'simulation/worker.py')],
                                        cwd=str(root), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, encoding='utf-8',
                                        errors='replace', bufsize=1, env=environment, **options)
        threading.Thread(target=self._read, daemon=True, name='duck-simulation-state').start()
        threading.Thread(target=self._read_errors, daemon=True, name='duck-simulation-errors').start()
        try:
            self.command({'op': 'load', 'model_id': model_id, 'expected_sha':expected_sha}, timeout=15)
            deadline = time.monotonic() + 5
            with self.condition:
                while not (self.state and self.state.get('loaded') and self.state.get('modelSha') == expected_sha
                           and len(self.state.get('body_poses', [])) == 15):
                    if self.closed or self.process.poll() is not None:
                        detail = ('：' + self.errors[-1]) if self.errors else ''
                        raise RuntimeError('仿真进程已结束' + detail)
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('仿真模型状态未就绪')
                    self.condition.wait(remaining)
        except Exception:
            self.close()
            raise

    def _read(self):
        try:
            for line in self.process.stdout:
                if len(line) > 65536:
                    raise ValueError('仿真状态过大')
                value = json.loads(line)
                with self.condition:
                    if value.get('type') == 'state':
                        self.state = value
                    elif value.get('type') == 'ack':
                        self.acks[value.get('id')] = value
                        if len(self.acks) > 64:
                            self.acks.pop(next(iter(self.acks)))
                    elif value.get('type') == 'error':
                        self.errors.append(str(value.get('error', '仿真失败'))[:500])
                    self.condition.notify_all()
        except (OSError, ValueError) as error:
            with self.condition:
                if not self.closed:
                    self.errors.append(str(error)[:500])
        finally:
            with self.condition:
                self.condition.notify_all()

    def _read_errors(self):
        try:
            for line in self.process.stderr:
                with self.condition:
                    self.errors.append(line.strip()[:500])
        except OSError:
            pass

    def snapshot(self):
        with self.condition:
            if self.closed or self.process.poll() is not None:
                raise ValueError('仿真会话已结束，请重新打开模型')
            result = dict(self.state or {'loaded': False, 'paused': True})
            result['modelId'] = self.model_id
            if self.errors:
                result['workerMessage'] = self.errors[-1]
            return result

    def command(self, payload, timeout=5):
        with self.command_lock:
            with self.condition:
                if self.closed or self.process.poll() is not None:
                    raise ValueError('仿真会话已结束，请重新打开模型')
                self.serial += 1
                command_id = self.serial
                command = dict(payload, id=command_id)
                deadline = time.monotonic() + timeout
                self.process.stdin.write(json.dumps(command, allow_nan=False) + '\n')
                self.process.stdin.flush()
                while command_id not in self.acks:
                    if self.closed or self.process.poll() is not None:
                        detail = ('：' + self.errors[-1]) if self.errors else ''
                        raise RuntimeError('仿真进程已结束' + detail)
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        # Unknown outcome is never automatically replayed.
                        raise TimeoutError('仿真命令超时；该操作未自动重试')
                    self.condition.wait(min(remaining, .5))
                ack = self.acks.pop(command_id)
                if not ack.get('ok'):
                    raise ValueError(str(ack.get('error', '仿真命令失败'))[:500])
                # worker writes the command's state immediately after its ack.
                old_seq = (self.state or {}).get('seq', -1)
                while (self.state or {}).get('seq', -1) <= old_seq and time.monotonic() < deadline:
                    self.condition.wait(min(.1, max(0, deadline - time.monotonic())))
                if self.subscribers == 0:
                    self.idle_since = time.monotonic()
                    self.idle_paused = False
                return self.snapshot()

    def subscribe(self):
        with self.condition:
            if self.closed:
                raise ValueError('仿真会话已结束')
            self.subscribers += 1

    def unsubscribe(self):
        with self.condition:
            self.subscribers = max(0, self.subscribers - 1)
            if not self.subscribers:
                self.idle_since = time.monotonic()
                self.idle_paused = False

    def wait_state(self, after_seq, timeout=1):
        with self.condition:
            self.condition.wait_for(lambda: self.closed or self.process.poll() is not None
                                    or (self.state or {}).get('seq', -1) > after_seq, timeout)
            return self.snapshot()

    def close(self):
        with self.condition:
            if self.closed:
                return
            self.closed = True
            self.condition.notify_all()
        with self.command_lock:
            try:
                self.process.stdin.close()
            except (OSError, ValueError):
                pass
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            for pipe in (self.process.stdout, self.process.stderr):
                try:
                    pipe.close()
                except OSError:
                    pass


class SimulationService:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.lock = threading.RLock()
        self.sessions = {}
        self.pending = 0
        self.stopped = threading.Event()
        threading.Thread(target=self._clean, daemon=True, name='duck-simulation-cleanup').start()
        atexit.register(self.close_all)

    def create(self, model_id):
        from simulation_catalog import resolve_model
        model = resolve_model(self.root, model_id)
        with self.lock:
            if len(self.sessions) + self.pending >= 1:
                raise ValueError('请先关闭当前仿真，再切换模型')
            self.pending += 1
        try:
            session = SimulationSession(self.root, model_id, model['sha256'])
            with self.lock:
                self.sessions[session.id] = session
            return session
        finally:
            with self.lock:
                self.pending -= 1

    def get(self, session_id):
        if not isinstance(session_id, str) or len(session_id) > 80:
            raise ValueError('仿真会话无效')
        with self.lock:
            session = self.sessions.get(session_id)
        if not session or session.closed:
            raise ValueError('仿真会话已结束，请重新打开模型')
        return session

    def close(self, session_id):
        with self.lock:
            session = self.sessions.pop(session_id, None)
        if session:
            session.close()

    def close_all(self):
        self.stopped.set()
        with self.lock:
            sessions = list(self.sessions.values())
            self.sessions.clear()
        for session in sessions:
            session.close()

    def _clean(self):
        while not self.stopped.wait(.25):
            with self.lock:
                sessions = list(self.sessions.values())
            for session in sessions:
                with session.condition:
                    idle = time.monotonic() - session.idle_since
                    pause = session.subscribers == 0 and idle > 2 and not session.idle_paused
                    expired = session.subscribers == 0 and idle > 30
                if expired or session.closed or session.process.poll() is not None:
                    self.close(session.id)
                elif pause:
                    try:
                        session.command({'op': 'pause', 'paused': True}, timeout=2)
                        with session.condition:
                            # Internal pause does not extend the cleanup deadline.
                            session.idle_since = time.monotonic() - idle
                            session.idle_paused = True
                    except (ValueError, RuntimeError, OSError, TimeoutError):
                        self.close(session.id)
