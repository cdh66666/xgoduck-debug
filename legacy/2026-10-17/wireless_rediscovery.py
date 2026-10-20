"""One bounded, read-only rediscovery job; authenticated cache updates only."""
from device_config import DEVICE_SERIAL
import json
import threading
import time
from pathlib import Path


class AutomaticRediscovery:
    def __init__(self, root, allowed, commit, clock=time.monotonic, finder=None):
        self.root, self.allowed, self.commit = Path(root), allowed, commit
        self.clock, self.finder = clock, finder
        self.lock = threading.Lock()
        self.thread = None
        self.failures = 0
        self.next_at = 0
        self.state = 'idle'

    def snapshot(self):
        with self.lock:
            return {'state': self.state, 'active': self.thread is not None,
                    'retry_in_s': round(max(0, self.next_at - self.clock())),
                    'read_only': True}

    def request(self):
        # Never create a second worker from high frequency telemetry pollers.
        if not self.allowed():
            return False
        with self.lock:
            if self.thread is not None or self.clock() < self.next_at:
                return False
            self.state = 'searching'
            self.thread = threading.Thread(target=self._run, name='xgoduck-rediscovery', daemon=True)
            self.thread.start()
            return True

    def _find(self):
        import sys
        path = str(self.root / 'tools')
        if path not in sys.path:
            sys.path.insert(0, path)
        import discover_unoq as discovery
        from deploy_readonly_wifi import strict_identity
        identity = strict_identity(self.root)
        if identity.get('serial') != DEVICE_SERIAL:
            raise ValueError('unbound controller')
        return discovery.discover_pinned(root=self.root, identity_loader=lambda _: identity,
            should_continue=self.allowed, emit=lambda _: None)

    def _run(self):
        success = False
        try:
            if self.allowed():
                result = (self.finder or self._find)()
                if self.allowed():
                    success = self.commit(result) is True
        except Exception:
            pass  # No raw SSH/identity details appear in public state.
        finally:
            with self.lock:
                self.failures = 0 if success else min(self.failures + 1, 5)
                self.next_at = self.clock() + (30 if success else min(300, 30 * 2 ** (self.failures - 1)))
                self.state = 'verified' if success else 'waiting'
                self.thread = None


def verified_result(result):
    if not isinstance(result, dict) or result.get('serial') != DEVICE_SERIAL:
        return False
    evidence = result.get('connection_evidence', result.get('discovery_evidence'))
    return (isinstance(evidence, dict) and evidence.get('ssh_pin_verified') is True
            and evidence.get('sys_serial') == DEVICE_SERIAL
            and evidence.get('counters_advanced') is True
            and result.get('stage') == 'complete')


def persist_verified(root, result):
    """Caller must serialize with actions/update interlock checks."""
    if not verified_result(result):
        return False
    root = Path(root)
    path = root / 'qa' / 'discovery-last.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name('discovery-last.auto.tmp')
    temporary.write_text(json.dumps({**result, 'automatic': True}, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)
    return True
