"""Exercise the real read-only remote source against temporary local fixtures."""
import base64
import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'pc/tools'))
import fetch_robot_models as fetch


class ModelFetchTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='offline-model-fetch-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repo = self.root/'repo'
        self.app = self.root/'duck-app'
        (self.repo/'pc').mkdir(parents=True)
        (self.app/'python').mkdir(parents=True)
        self.data = {name: (name.encode()*9000 if name == 'xgoduck_walk.onnx' else name.encode()*20)
                     for name in fetch.NAMES}
        self.manifest = {'external_models': {'python/'+name: {'path': 'robot-app/python/'+name,
                         'sha256': hashlib.sha256(raw).hexdigest()} for name, raw in self.data.items()}}
        self.save_manifest()
        for name, raw in self.data.items():
            (self.app/'python'/name).write_bytes(raw)
        self.calls = []

    def save_manifest(self):
        (self.repo/'RELEASE_RUNTIME.json').write_text(json.dumps(self.manifest), 'utf-8')

    def run_source(self, payload):
        source = fetch.REMOTE_SOURCE.replace(repr(fetch.REMOTE_APP), repr(str(self.app)))
        output = io.StringIO()
        with patch.object(sys, 'stdin', io.StringIO(json.dumps(payload))), contextlib.redirect_stdout(output):
            exec(compile(source, '<offline-readonly-model-source>', 'exec'), {})
        return json.loads(output.getvalue())

    def transport(self, root, source, payload, timeout):
        self.assertEqual(root, self.repo/'pc')
        self.assertEqual(source, fetch.REMOTE_SOURCE)
        self.assertEqual(timeout, 20)
        self.calls.append(payload)
        result = self.run_source(payload)
        if 'data' in result:
            self.assertLessEqual(len(result['data']), 65536)
        return result

    def assert_no_parts(self):
        self.assertEqual(list(self.repo.rglob('*.part')), [])

    def test_four_models_download_in_bounded_chunks(self):
        result = fetch.fetch_models(self.repo, self.transport)
        self.assertEqual(set(result), set(self.manifest['external_models']))
        for name, raw in self.data.items():
            self.assertEqual((self.repo/'robot-app/python'/name).read_bytes(), raw)
        self.assertEqual(self.calls[0], {'operation': 'metadata'})
        self.assertEqual(sum(c['operation'] == 'metadata' for c in self.calls), 1)
        self.assertTrue(all(c['operation'] in ('metadata', 'read_chunk') for c in self.calls))
        self.assertGreater(sum(c.get('name') == 'xgoduck_walk.onnx' for c in self.calls), 1)
        self.assert_no_parts()

    def test_wrong_release_sha_rejected_before_download(self):
        self.manifest['external_models']['python/xgoduck_walk.onnx']['sha256'] = '0'*64
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'differs from this release'):
            fetch.fetch_models(self.repo, self.transport)
        self.assertEqual(len(self.calls), 1)
        self.assert_no_parts()

    def test_oversize_metadata_rejected(self):
        def oversize(*args, **kwargs):
            value = self.transport(*args, **kwargs)
            value['models']['xgoduck_walk.onnx']['size'] = fetch.MAX_BYTES+1
            return value
        with self.assertRaisesRegex(ValueError, '4MiB'):
            fetch.fetch_models(self.repo, oversize)
        self.assertEqual(len(self.calls), 1)
        self.assert_no_parts()

    def test_illegal_manifest_paths_and_model_set_are_offline_rejected(self):
        original = json.loads(json.dumps(self.manifest))
        for path in ('../escaped.onnx', '/tmp/model.onnx', 'robot-app/python/../xgoduck_walk.onnx'):
            with self.subTest(path=path):
                self.manifest = json.loads(json.dumps(original))
                self.manifest['external_models']['python/xgoduck_walk.onnx']['path'] = path
                self.save_manifest()
                with self.assertRaisesRegex(ValueError, 'path/SHA256'):
                    fetch.fetch_models(self.repo, self.transport)
        self.manifest = original
        self.manifest['external_models']['python/extra.onnx'] = {'path': 'robot-app/python/extra.onnx', 'sha256': '0'*64}
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'exactly the four'):
            fetch.fetch_models(self.repo, self.transport)
        self.assertEqual(self.calls, [])

    def test_remote_rejects_arbitrary_path_and_operations(self):
        for name in ('../data/zero_pos.json', '/etc/passwd', 'extra.onnx'):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'filename'):
                self.run_source({'operation': 'read_chunk', 'name': name, 'offset': 0, 'length': 1, 'size': 1})
        with self.assertRaisesRegex(ValueError, 'only metadata'):
            self.run_source({'operation': 'restart'})
        with self.assertRaisesRegex(ValueError, 'chunk request'):
            self.run_source({'operation': 'read_chunk', 'name': 'xgoduck_walk.onnx', 'offset': 0,
                             'length': 65537, 'size': len(self.data['xgoduck_walk.onnx'])})

    def test_remote_rejects_oversized_model(self):
        path = self.app/'python/xgoduck_walk.onnx'
        with path.open('wb') as stream:
            stream.truncate(fetch.MAX_BYTES+1)
        with self.assertRaisesRegex(ValueError, '4MiB'):
            fetch.fetch_models(self.repo, self.transport)
        self.assert_no_parts()

    def test_changed_content_preserves_existing_file_and_cleans_partial(self):
        name = sorted(fetch.NAMES)[0]
        destination = self.repo/'robot-app/python'/name
        destination.parent.mkdir(parents=True)
        destination.write_bytes(b'existing verified copy')
        def corrupt(*args, **kwargs):
            value = self.transport(*args, **kwargs)
            if 'data' in value:
                raw = base64.b64decode(value['data'])
                value['data'] = base64.b64encode(b'X'+raw[1:]).decode()
            return value
        with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
            fetch.fetch_models(self.repo, corrupt)
        self.assertEqual(destination.read_bytes(), b'existing verified copy')
        self.assert_no_parts()

    def test_truncated_chunk_and_transport_failure_clean_partial(self):
        for failure in ('truncate', 'transport'):
            def broken(*args, **kwargs):
                value = self.transport(*args, **kwargs)
                if 'data' in value:
                    if failure == 'transport':
                        raise RuntimeError('fake connection closed')
                    value['data'] = base64.b64encode(b'').decode()
                return value
            with self.subTest(failure=failure), self.assertRaises((ValueError, RuntimeError)):
                fetch.fetch_models(self.repo, broken)
            self.assert_no_parts()

    def test_default_and_help_do_not_load_transport(self):
        with patch.object(fetch, 'fetch_models', side_effect=AssertionError('must stay offline')), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(fetch.main([]), 0)
            with self.assertRaises(SystemExit) as result:
                fetch.main(['--help'])
            self.assertEqual(result.exception.code, 0)


if __name__ == '__main__':
    unittest.main()
