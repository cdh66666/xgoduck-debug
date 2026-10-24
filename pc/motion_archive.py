"""Archive robot-local episodes through the existing tunnel, using GET only."""
import gzip
import http.client
import json
from pathlib import Path
import re
import threading
import time

SCHEMA = 'xgoduck-motion-v1'
SESSION_ID = re.compile(r'^[a-zA-Z0-9_-]{1,100}$')


def read_episode(path):
    header, summary, frames = None, None, []
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('type') == 'header':
                header = row
            elif row.get('type') == 'frame':
                frames.append(row)
            elif row.get('type') == 'end':
                summary = row['summary']
    if not header or header.get('schema') != SCHEMA or not summary or not frames:
        raise ValueError('动作记录尚未完整保存')
    if header.get('id') != summary.get('id') or not SESSION_ID.fullmatch(summary.get('id', '')):
        raise ValueError('动作记录编号不匹配')
    return dict(summary=summary, frames=frames)


def save_episode(folder, summary, frames):
    """Also used to import real, explicitly labelled historical QA captures."""
    sid = summary['id']
    if not SESSION_ID.fullmatch(sid):
        raise ValueError('Invalid session id')
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    target = folder/(sid+'.jsonl.gz')
    part = target.with_suffix('.part')
    with gzip.open(part, 'wt', encoding='utf-8', compresslevel=1) as stream:
        stream.write(json.dumps(dict(type='header', **summary), ensure_ascii=False)+'\n')
        for row in frames:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n')
        stream.write(json.dumps(dict(type='end', summary=summary), ensure_ascii=False)+'\n')
    part.replace(target)
    (folder/(sid+'.json')).write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')


class MotionArchive:
    def __init__(self, root, tunnel, *, start=True):
        self.folder = Path(root)/'motion-recordings'
        self.folder.mkdir(parents=True, exist_ok=True)
        self.tunnel = tunnel
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.remote_items = []
        self.recorder = None
        self.error = ''
        self.last_sync = None
        self.thread = threading.Thread(target=self._run, name='motion-archive', daemon=True)
        if start:
            self.thread.start()

    def _get(self, path, limit):
        if self.tunnel.port is None:
            raise ConnectionError('主控未连接')
        connection = http.client.HTTPConnection('127.0.0.1', self.tunnel.port, timeout=5)
        try:
            connection.request('GET', path)
            response = connection.getresponse()
            if response.status != 200:
                raise ConnectionError('动作记录接口暂不可用 (%s)' % response.status)
            data = response.read(limit+1)
            if len(data) > limit:
                raise ValueError('动作记录超过单段读取范围')
            return data
        finally:
            connection.close()

    def sync(self):
        report = json.loads(self._get('/api/motion/sessions', 4*1024*1024))
        items = report['items']
        with self.lock:
            self.remote_items, self.recorder = items, report.get('recorder')
        for meta in items:
            sid = meta.get('id', '')
            if meta.get('state') == 'recording' or not SESSION_ID.fullmatch(sid):
                continue
            if meta.get('frames', 0) <= 0:
                meta['archive_note'] = '进程在首帧前中断，未保存姿态；不影响后续动作同步'
                continue
            target = self.folder/(sid+'.jsonl.gz')
            if target.exists() and (self.folder/(sid+'.json')).exists():
                continue
            data = self._get('/api/motion/sessions/'+sid, 32*1024*1024)
            part = target.with_suffix('.part')
            try:
                part.write_bytes(data)
                episode = read_episode(part)
                if episode['summary']['id'] != sid:
                    raise ValueError('下载动作编号不匹配')
                part.replace(target)
                metadata = self.folder/(sid+'.json')
                temporary = metadata.with_suffix('.json.part')
                temporary.write_text(json.dumps(episode['summary'], ensure_ascii=False), encoding='utf-8')
                temporary.replace(metadata)
            finally:
                part.unlink(missing_ok=True)
        with self.lock:
            self.error, self.last_sync = '', time.time()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                self.sync()
            except (OSError, EOFError, ValueError, KeyError, http.client.HTTPException) as error:
                with self.lock:
                    self.error = str(error)
            self.stop_event.wait(1)

    def listing(self):
        local = {}
        for path in self.folder.glob('*.json'):
            try:
                meta = json.loads(path.read_text('utf-8'))
                local[meta['id']] = dict(meta, archived=True)
            except (OSError, ValueError, KeyError):
                continue
        with self.lock:
            for meta in self.remote_items:
                if meta['id'] not in local:
                    local[meta['id']] = dict(meta, archived=False)
            return dict(ok=True, items=sorted(local.values(), key=lambda r: r.get('start_unix_s', 0), reverse=True),
                        recorder=self.recorder, error=self.error, last_sync_unix_s=self.last_sync)

    def path(self, sid):
        if not SESSION_ID.fullmatch(sid):
            raise ValueError('Invalid session id')
        path = self.folder/(sid+'.jsonl.gz')
        if not path.is_file():
            raise FileNotFoundError('动作尚未同步到电脑')
        return path

    def episode(self, sid):
        return read_episode(self.path(sid))

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(6)
