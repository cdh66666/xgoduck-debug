"""Exercise the read-only stream using a local chunked HTTP/1.1 server."""
import concurrent.futures
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from official_status_stream import OfficialStatusStream


def state(sequence, uptime=None):
    return {'deployment': {'reference': 'rl/raspi_deploy'}, 'mcu_seq': sequence,
            'uptime_s': sequence if uptime is None else uptime,
            'telemetry_sent_unix_s': 1}


class LocalStream:
    def __init__(self, content_type='application/x-ndjson'):
        self.frames = queue.Queue()
        self.paths = []
        self.connected = threading.Event()
        self.stopped = threading.Event()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *args):
                pass

            def do_GET(self):
                owner.paths.append(('GET', self.path))
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Transfer-Encoding', 'chunked')
                self.end_headers()
                owner.connected.set()
                try:
                    while not owner.stopped.is_set():
                        try:
                            frame = owner.frames.get(timeout=.05)
                        except queue.Empty:
                            continue
                        if frame is None:
                            self.wfile.write(b'0\r\n\r\n')
                            self.wfile.flush()
                            return
                        raw = json.dumps(frame).encode()+b'\n'
                        self.wfile.write(f'{len(raw):x}\r\n'.encode()+raw+b'\r\n')
                        self.wfile.flush()
                except OSError:
                    pass
                self.close_connection = True

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_port
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def put(self, sequence, uptime=None):
        self.frames.put(state(sequence, uptime))

    def close(self):
        self.stopped.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.server = LocalStream()
        self.addCleanup(self.server.close)
        self.tunnel = SimpleNamespace(port=self.server.port, error=None, official=False)
        self.reader = OfficialStatusStream(self.tunnel)
        self.addCleanup(self.reader.close)
        self.assertTrue(self.server.connected.wait(1))

    def wait_for(self, predicate):
        with self.reader._condition:
            self.assertTrue(self.reader._condition.wait_for(predicate, timeout=1))

    def publish(self, sequence, uptime=None):
        before = self.reader._generation
        self.server.put(sequence, uptime)
        self.wait_for(lambda: self.reader._generation > before)
        return self.reader.get()

    def test_multiple_frames_use_one_get_and_real_pc_receive_times(self):
        self.publish(1)
        value = self.publish(2)
        self.assertEqual(value['mcu_seq'], 2)
        self.assertEqual(value['pc_transport'], 'official-status-stream')
        self.assertGreater(value['pc_observed_unix_s'], value['telemetry_sent_unix_s'])
        self.assertEqual(value['pc_observed_monotonic_s'], self.reader.read_at)
        value['deployment']['reference'] = 'changed'
        self.assertEqual(self.reader.get()['deployment']['reference'], 'rl/raspi_deploy')
        self.assertEqual(self.server.paths, [('GET', '/api/status-stream')])

    def test_duplicate_sequence_does_not_refresh_timestamp(self):
        self.publish(1, 10)
        observed = self.reader.read_at
        generation = self.reader._generation
        self.server.put(1, 11)
        self.wait_for(lambda: self.reader.last_uptime == 11)
        self.assertEqual(self.reader.read_at, observed)
        self.assertEqual(self.reader._generation, generation)
        with self.assertRaises((TimeoutError, OSError)):
            self.reader.get()

    def test_get_ages_cache_but_preserves_actual_history_endpoint(self):
        self.publish(1,10)
        with self.reader._condition:
            self.reader._value.update(feedback_age_ms=8.,full_robot_ready=True)
            self.reader.read_at-=.25
        value=self.reader.get()
        self.assertGreaterEqual(value['feedback_age_ms'],258.)
        self.assertEqual(value['native_feedback_age_ms'],8.)
        self.assertFalse(value['full_robot_ready'])
        self.assertEqual(self.reader.value['feedback_age_ms'],8.)

    def test_sub_350ms_burst_is_retained_as_history_but_not_current_readiness(self):
        first=dict(state(1,100),feedback_age_ms=8.,telemetry_cache_age_ms=0.,full_robot_ready=True)
        self.reader._accept(first,self.tunnel.port,1000,1000)
        following=dict(state(2,100.05),feedback_age_ms=8.,telemetry_cache_age_ms=0.,full_robot_ready=True)
        self.reader._accept(following,self.tunnel.port,1000.35,1000.35)
        from status_age import aged_snapshot
        raw=self.reader.value
        current=aged_snapshot(raw,1000.35,1000.35)
        self.assertAlmostEqual(raw['pc_delivery_age_lower_bound_ms'],300.)
        self.assertEqual(raw['feedback_age_ms'],8.)
        self.assertTrue(raw['full_robot_ready'])
        self.assertAlmostEqual(current['feedback_age_ms'],308.)
        self.assertFalse(current['full_robot_ready'])
        self.assertEqual(self.reader._generation,2)

    def test_continuous_backlog_cannot_renew_after_reference_window_expires(self):
        self.assertTrue(self.reader._accept(state(1,100),self.tunnel.port,1000,1000))
        with self.assertRaises(ConnectionError):self.reader._accept(state(2,100.05),self.tunnel.port,1001,1001)
        generation=self.reader._generation
        with self.assertRaises(ConnectionError):self.reader._accept(state(3,101.1),self.tunnel.port,1010.1,1010.1)
        self.assertEqual(self.reader._generation,generation)
        self.assertEqual(self.reader.value['mcu_seq'],1)
        self.assertTrue(self.reader._accept(state(4,110.11),self.tunnel.port,1010.12,1010.12))
        self.assertIsNone(self.reader._backlog_reference)
        self.assertEqual(self.reader.value['mcu_seq'],4)

    def test_receiver_resync_starts_new_display_epoch_without_renewing_network_backlog(self):
        first = state(275438, 100)
        first['feedback_timing'] = {'receiver_epoch': 1}
        self.reader._accept(first, self.tunnel.port, 1000, 1000)
        epoch = self.reader.value['pc_host_epoch']
        following = state(275396, 100.05)
        following['feedback_timing'] = {'receiver_epoch': 2}
        self.reader._accept(following, self.tunnel.port, 1000.05, 1000.05)
        self.assertEqual(self.reader.value['pc_host_epoch'], epoch+1)
        self.assertEqual(self.reader.value['mcu_seq'], 275396)
        self.assertEqual(len(self.reader._frames), 1)
        late = state(275397, 100.1)
        late['feedback_timing'] = {'receiver_epoch': 3}
        with self.assertRaises(ConnectionError):
            self.reader._accept(late, self.tunnel.port, 1001.1, 1001.1)
        self.assertEqual(self.reader.value['mcu_seq'], 275396)

    def test_disconnect_fails_immediately_and_retains_history(self):
        self.publish(1)
        self.server.frames.put(None)
        self.wait_for(lambda: self.reader._failure is not None)
        before = time.monotonic()
        with self.assertRaises(ConnectionError):
            self.reader.get()
        self.assertLess(time.monotonic()-before, .05)
        self.assertEqual(self.reader.value['mcu_seq'], 1)
        self.assertFalse(self.tunnel.official)
        self.assertIsNotNone(self.tunnel.error)

    def test_stale_frame_fails_after_350ms(self):
        self.publish(1)
        with self.reader._condition:
            self.reader._condition.wait_for(lambda: self.reader._failure is not None, timeout=.6)
        self.assertGreaterEqual(time.monotonic()-self.reader.read_at, .35)
        with self.assertRaises((TimeoutError, OSError)):
            self.reader.get()

    def test_brief_transport_gap_waits_for_a_new_fresh_frame(self):
        self.publish(1)
        with self.reader._condition:
            self.reader.read_at -= .4
        timer=threading.Timer(.05,lambda:self.server.put(2))
        timer.start()
        self.addCleanup(timer.join)
        started=time.monotonic()
        value=self.reader.get()
        self.assertEqual(value['mcu_seq'],2)
        self.assertGreater(time.monotonic()-started,.03)
        self.assertLess(time.monotonic()-self.reader.read_at,.35)
        self.assertEqual(self.server.paths,[('GET','/api/status-stream')])

    def test_invalidation_waits_for_next_real_frame(self):
        self.publish(1)
        self.reader.value = None
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(self.reader.get)
            self.server.put(1, 2)
            with self.reader._condition:
                self.reader._condition.wait_for(lambda: self.reader.last_uptime == 2, timeout=.05)
            self.assertFalse(pending.done())
            self.server.put(2, 3)
            self.assertEqual(pending.result(1)['mcu_seq'], 2)

    def test_port_change_waits_for_new_stream_without_new_epoch(self):
        self.publish(1, 10)
        other = LocalStream()
        self.addCleanup(other.close)
        self.tunnel.port = other.port
        with self.assertRaises(ConnectionError):
            self.reader.get()
        other.put(2, 11)
        self.wait_for(lambda: self.reader._value['mcu_seq'] == 2)
        self.assertEqual(self.reader.get()['pc_host_epoch'], 1)
        self.assertTrue(self.tunnel.official)
        self.assertEqual(other.paths, [('GET', '/api/status-stream')])

    def test_reconnect_does_not_increment_epoch_but_host_restart_does(self):
        self.publish(1, 10)
        self.server.frames.put(None)
        self.wait_for(lambda: self.reader._failure is not None)
        self.assertEqual(self.publish(2, 11)['pc_host_epoch'], 1)
        self.assertEqual(self.publish(2, 1)['pc_host_epoch'], 2)

    def test_close_unblocks_reader_and_leaves_no_worker(self):
        self.publish(1)
        self.reader.value = None
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(self.reader.get)
            self.reader.close()
            with self.assertRaises(ConnectionError):
                pending.result(1)
        self.assertFalse(self.reader._worker.is_alive())
        with self.assertRaises(ConnectionError):
            self.reader.get()

    def test_batched_endpoints_are_retained_without_polling_between_frames(self):
        for sequence in range(1, 5):
            self.server.put(sequence)
        self.wait_for(lambda: self.reader._generation == 4)
        frames = self.reader.frames()
        self.assertEqual([frame['mcu_seq'] for frame in frames], [1, 2, 3, 4])
        self.assertEqual(self.reader.get()['mcu_seq'], 4)
        self.assertTrue(all(frame['pc_transport'] == 'official-status-stream' for frame in frames))
        frames[0]['deployment']['reference'] = 'changed'
        self.assertEqual(self.reader.frames()[0]['deployment']['reference'], 'rl/raspi_deploy')

    def test_history_is_bounded_to_64_real_frames_and_one_second(self):
        now = time.monotonic()
        for sequence in range(1, 71):
            self.reader._accept(state(sequence), self.tunnel.port, now, time.time())
        self.assertEqual([frame['mcu_seq'] for frame in self.reader.frames()], list(range(7, 71)))
        with self.reader._condition:
            for frame in self.reader._frames:
                frame['pc_observed_monotonic_s'] = now-2
        self.assertEqual(self.reader.frames(), [])

    def test_duplicate_and_invalidation_do_not_invent_history_frames(self):
        self.publish(1, 10)
        self.reader.value = None
        self.reader._accept(state(1, 11), self.tunnel.port, time.monotonic(), time.time())
        self.assertEqual(len(self.reader.frames()), 1)
        self.assertEqual(self.reader._generation, 1)
        self.reader._accept(state(1, 1), self.tunnel.port, time.monotonic(), time.time())
        self.assertEqual(len(self.reader.frames()), 1)
        self.assertEqual(self.reader.frames()[0]['pc_host_epoch'], 2)

    def test_history_on_disconnect_does_not_make_get_successful(self):
        self.publish(1)
        self.server.frames.put(None)
        self.wait_for(lambda: self.reader._failure is not None)
        self.assertEqual(len(self.reader.frames()), 1)
        with self.assertRaises(ConnectionError):
            self.reader.get()

    def test_history_cursor_returns_only_new_observed_endpoints(self):
        now=time.perf_counter()
        for sequence in range(1,4):
            self.reader._accept(state(sequence),self.tunnel.port,now+sequence*.01,time.time())
        self.assertEqual([v['mcu_seq'] for v in self.reader.frames(after=now+.02)],[3])
        self.assertEqual(self.reader.frames(after=now+1),[])
        self.assertEqual([v['mcu_seq'] for v in self.reader.frames()],[1,2,3])

    def test_observation_cursor_preserves_frames_with_identical_received_time(self):
        now=time.perf_counter()
        for sequence in range(1,4):
            self.reader._accept(state(sequence),self.tunnel.port,now,time.time())
        self.assertEqual([v['mcu_seq'] for v in self.reader.frames(after_observation=1)],[2,3])
        self.assertEqual([v['pc_observation_id'] for v in self.reader.frames()],[1,2,3])
        self.assertEqual([v['mcu_seq'] for v in self.reader.frames(after_observation=1,through_observation=2)],[2])
        self.reader._accept(state(3),self.tunnel.port,now+.001,time.time())
        self.assertEqual(self.reader.get()['pc_observation_id'],3)


class ContentTypeTests(unittest.TestCase):
    def test_rejects_non_ndjson_response(self):
        server = LocalStream('application/json')
        reader = OfficialStatusStream(SimpleNamespace(port=server.port, error=None, official=False))
        try:
            with reader._condition:
                self.assertTrue(reader._condition.wait_for(lambda: reader._failure is not None, timeout=1))
            with self.assertRaisesRegex(ValueError, 'NDJSON'):
                reader.get()
        finally:
            reader.close()
            server.close()


if __name__ == '__main__':
    unittest.main()
