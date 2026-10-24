"""Offline evidence for automatic boundaries, persistence and GET-only archival."""
import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'robot-app/python'), str(ROOT/'pc')]
from motion_recording import MotionRecorder
from motion_archive import MotionArchive, read_episode


def state(t, mode='off', enabled=False):
    return dict(uptime_s=t, mode=mode, enabled=enabled, gravity=[0, 0, -1],
                angles=[0]*15, targets=[0]*15, policy_targets=[0]*15, command=[0, 0, 0],
                recovery=dict(phase='walk'), feedback_age_ms=2, servo_ids=list(range(15)),
                gyro=[0, 0, 0], last_error='')


class MotionRecordingTests(unittest.TestCase):
    def make(self, directory):
        return MotionRecorder(lambda: None, directory, start=False, provenance={'sha256': {'controller.py': 'test'}})

    def episode(self, recorder):
        return read_episode(next(recorder.directory.glob('*.jsonl.gz')))

    def test_idle_does_not_create_episode(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            for i in range(100):
                recorder.consume(state(i*.04), 1000+i*.04)
            self.assertEqual(list(Path(folder).iterdir()), [])
            self.assertLessEqual(len(recorder.pre), 25)

    def test_automatic_preroll_control_fields_and_disabled_postroll(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            for i in range(25):
                recorder.consume(state(i*.04), 1000+i*.04)
            recorder.consume(state(1, 'policy', False), 1001)
            recorder.consume(state(1.04, 'policy', True), 1001.04)
            recorder.consume(state(1.08, 'off', True), 1001.08)
            recorder.consume(state(1.12), 1001.12)
            recorder.consume(state(2.12), 1002.12)
            report = self.episode(recorder)
            self.assertIsNone(recorder.active)
            self.assertEqual(report['summary']['end_reason'], 'disabled')
            self.assertLess(report['frames'][0]['t'], 0)
            self.assertFalse(report['frames'][-1]['status']['enabled'])
            self.assertEqual(report['frames'][26]['status']['policy_targets'], [0]*15)
            self.assertEqual(report['summary']['provenance']['sha256']['controller.py'], 'test')
            self.assertGreater(report['summary']['sample_gaps'], 0)

    def test_two_sessions_survive_recorder_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            for t in (1, 4):
                recorder.consume(state(t, 'policy', True), 1000+t)
                recorder.consume(state(t+1.1), 1001.1+t)
            restarted = self.make(folder)
            self.assertEqual(len(restarted.sessions()), 2)
            self.assertTrue(all(row['state'] == 'complete' for row in restarted.sessions()))

    def test_one_axis_maintenance_is_recorded_even_without_global_enable(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            value = state(1, 'servo_debug')
            value['factory_calibration'] = {'held_mask': 1}
            recorder.consume(value, 1001)
            recorder.consume(state(2.1), 1002.1)
            self.assertEqual(self.episode(recorder)['summary']['label'], '校准 / 维护')

    def test_queue_overflow_is_visible_and_submit_copies_mutable_state(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            value = state(1)
            recorder.submit(value, 1001)
            value['angles'][0] = 99
            self.assertEqual(recorder.queue.get_nowait()[0]['angles'][0], 0)
            for i in range(513):
                recorder.submit(state(i), 1000+i)
            self.assertEqual(recorder.snapshot()['queue_drops'], 1)

    def test_short_accepted_start_is_retained_between_periodic_samples(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            recorder.status = lambda: state(1)
            recorder.mark_event('mode', {'mode': 'policy'})
            value, wall = recorder.queue.get_nowait()
            self.assertTrue(value['control_event']['accepted'])
            recorder.consume(value, wall)
            recorder.consume(state(2.1), wall+1.1)
            episode = self.episode(recorder)
            self.assertEqual(episode['frames'][0]['status']['control_event']['body']['mode'], 'policy')
            self.assertTrue(any('control_event' in event for event in episode['summary']['events']))

    def test_stream_reuses_source_snapshot_and_ages_the_cache_without_renewing_time(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            value = state(1)
            value.update(control_ready=True, full_robot_ready=True, joint_feedback_age_ms=[2]*15,
                         display_heading=dict(age_ms=3,available=True),
                         servo_current=dict(age_ms=3,available=True,sum_abs_a=1))
            recorder.status = lambda: value
            with patch('motion_recording.time.monotonic', return_value=10):
                recorder.capture()
            value['angles'][0]=99
            value['recovery']['phase']='getup'
            with patch('motion_recording.time.monotonic', return_value=10.2):
                cached = recorder.latest()
            self.assertEqual(cached['uptime_s'], 1)
            self.assertEqual(cached['angles'][0],0)
            self.assertEqual(cached['recovery']['phase'],'walk')
            self.assertAlmostEqual(cached['feedback_age_ms'], 202)
            self.assertFalse(cached['full_robot_ready'])
            self.assertFalse(cached['control_ready'])
            self.assertFalse(cached['display_heading']['available'])
            self.assertTrue(cached['servo_current']['stale'])
            self.assertAlmostEqual(cached['joint_feedback_age_ms'][0],202)
            self.assertEqual(value['feedback_age_ms'],2)

    def test_stream_wakes_on_capture_and_skips_consumers_old_snapshots(self):
        import concurrent.futures
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            self.assertEqual(recorder.wait_latest(0, timeout=0), (0, None))
            recorder.status = lambda: state(1)
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                waiting = pool.submit(recorder.wait_latest, 0)
                recorder.capture()
                generation, value = waiting.result(timeout=1)
            self.assertEqual(generation, 1)
            self.assertEqual(value['uptime_s'], 1)
            for t in [2, 3, 4]:
                recorder.status = lambda: state(t)
                recorder.capture()
            generation, value = recorder.wait_latest(1, timeout=0)
            self.assertEqual(generation, 4)
            self.assertEqual(value['uptime_s'], 4)
            self.assertEqual(recorder.queue.qsize(), 4, 'recording still retains every capture')

    def test_partial_joint_expiry_cannot_leave_cached_control_ready_true(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            value = state(1)
            value.update(control_ready=True, full_robot_ready=True,
                         joint_feedback_age_ms=[149]+[2]*14,stale_joint_ids=[])
            recorder.status = lambda: value
            with patch('motion_recording.time.monotonic',return_value=10):recorder.capture()
            with patch('motion_recording.time.monotonic',return_value=10.02):cached=recorder.latest()
            self.assertFalse(cached['control_ready'])
            self.assertEqual(cached['stale_joint_ids'],[10])
            self.assertEqual(value['stale_joint_ids'],[])

    def test_crash_recovery_preserves_flushed_frames_without_claiming_normal_end(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            recorder.consume(state(1, 'policy', True), 1001)
            recorder.consume(state(2, 'policy', True), 1002)
            recorder.stream.flush()
            # A killed process never writes the gzip trailer.
            raw = next(Path(folder).glob('*.jsonl.gz')).read_bytes()
            recorder.stream.close()
            next(Path(folder).glob('*.jsonl.gz')).write_bytes(raw)
            recovered = self.make(folder)
            report = self.episode(recovered)
            self.assertEqual(report['summary']['state'], 'interrupted')
            self.assertEqual(report['summary']['end_reason'], 'process_interrupted')
            self.assertEqual(len(report['frames']), 2)

    def test_source_gap_is_preserved_and_large_tilt_is_marked_after_upright(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = self.make(folder)
            recorder.consume(state(1, 'policy', True), 1001)
            falling = state(1.5, 'policy', True)
            falling['gravity'] = [1, 0, 0]
            recorder.consume(falling, 1001.5)
            recorder.consume(state(2.6), 1002.6)
            report = self.episode(recorder)
            self.assertEqual(report['summary']['tilt_after_upright_at_s'], .5)
            self.assertGreater(report['summary']['sample_gaps'], 0)
            self.assertEqual([row['t'] for row in report['frames']], [0, .5, 1.6])

    def test_archive_download_is_complete_get_only_and_available_offline(self):
        with tempfile.TemporaryDirectory() as remote, tempfile.TemporaryDirectory() as local:
            recorder = self.make(remote)
            recorder.consume(state(1, 'policy', True), 1001)
            recorder.consume(state(2.1), 1002.1)
            sid = recorder.sessions()[0]['id']
            archive = MotionArchive(local, SimpleNamespace(port=None), start=False)
            paths = []
            def get(path, limit):
                paths.append(path)
                if path == '/api/motion/sessions':
                    return json.dumps(dict(items=recorder.sessions(), recorder=recorder.snapshot())).encode()
                return recorder.session_path(sid).read_bytes()
            archive._get = get
            archive.sync()
            self.assertEqual(paths, ['/api/motion/sessions', '/api/motion/sessions/'+sid])
            self.assertTrue(archive.listing()['items'][0]['archived'])
            restarted = MotionArchive(local, SimpleNamespace(port=None), start=False)
            self.assertEqual(restarted.episode(sid)['summary']['id'], sid)
            self.assertEqual(len(restarted.listing()['items']), 1)
            with self.assertRaises(ValueError):
                restarted.path('../../other')

    def test_partial_download_is_not_admitted_to_history(self):
        with tempfile.TemporaryDirectory() as remote, tempfile.TemporaryDirectory() as local:
            recorder = self.make(remote)
            recorder.consume(state(1, 'policy', True), 1001)
            recorder.consume(state(2.1), 1002.1)
            sid = recorder.sessions()[0]['id']
            archive = MotionArchive(local, SimpleNamespace(port=None), start=False)
            archive._get = lambda path, limit: (json.dumps(dict(items=recorder.sessions())).encode()
                if path.endswith('/sessions') else recorder.session_path(sid).read_bytes()[:-20])
            with self.assertRaises((EOFError, ValueError, OSError)):
                archive.sync()
            self.assertFalse((archive.folder/(sid+'.jsonl.gz')).exists())

    def test_empty_interrupted_record_does_not_block_the_following_valid_record(self):
        with tempfile.TemporaryDirectory() as remote, tempfile.TemporaryDirectory() as local:
            recorder = self.make(remote)
            recorder.consume(state(1, 'policy', True), 1001)
            recorder.consume(state(2.1), 1002.1)
            valid = recorder.sessions()[0]
            empty = dict(valid, id='interrupted-before-first-frame', state='interrupted', frames=0)
            archive = MotionArchive(local, SimpleNamespace(port=None), start=False)
            paths = []
            def get(path, limit):
                paths.append(path)
                return (json.dumps(dict(items=[empty, valid])).encode() if path.endswith('/sessions')
                        else recorder.session_path(valid['id']).read_bytes())
            archive._get = get
            archive.sync()
            self.assertEqual(paths, ['/api/motion/sessions', '/api/motion/sessions/'+valid['id']])
            self.assertEqual(archive.episode(valid['id'])['summary']['id'], valid['id'])
            self.assertIn('首帧前中断', next(row for row in archive.listing()['items'] if row['id']==empty['id'])['archive_note'])


if __name__ == '__main__':
    unittest.main()
