"""Public diagnostic evidence and uninstalled-candidate boundaries; no IO to devices."""
import hashlib
import json
from pathlib import Path
import re
import unittest

ROOT=Path(__file__).resolve().parents[1]
BASE='181d85760c6fb9be9ed89d96b6f77bc4a2017a183a6ebb3b530189dbc1e5ee78'

class AuditPublicationTests(unittest.TestCase):
    def test_candidates_never_replace_default_controller(self):
        actual=hashlib.sha256((ROOT/'robot-app/python/controller.py').read_bytes()).hexdigest()
        self.assertEqual(actual,BASE)
        runtime=json.loads((ROOT/'RELEASE_RUNTIME.json').read_text('utf-8'))
        self.assertEqual(runtime['runtime_sources']['python/controller.py']['sha256'],BASE)
        self.assertTrue(all(row['path'].startswith('robot-app/') for row in runtime['runtime_sources'].values()))
        candidate=ROOT/'candidates/host-feedback-20261020'
        manifest=json.loads((candidate/'MANIFEST.json').read_text('utf-8'))
        self.assertFalse(manifest['installed'])
        variants=[row for row in manifest['files'] if row['label']!='offline-test']
        self.assertEqual(len(variants),3)
        for row in variants:
            self.assertFalse(row['installed'])
            self.assertFalse(row['default_runtime_replaced'])
            self.assertEqual(hashlib.sha256((candidate/row['path']).read_bytes()).hexdigest(),row['source_sha256'])

    def test_anonymous_scope_is_six_diagnostic_cases_not_robot_admission(self):
        report=ROOT/'reports/standing-system-audit-20261020'
        data=json.loads((report/'scope-summary.json').read_text('utf-8'))
        self.assertEqual(len(data['tests']),6)
        self.assertFalse(data['real_robot_admission'])
        self.assertFalse(data['raw_traces_included'])
        self.assertFalse(data['exact_historical_training_environment_verified'])
        for path in report.glob('*.json'):
            text=path.read_text('utf-8')
            self.assertNotRegex(text,r'[A-Za-z]:[\\/]')
            self.assertNotRegex(text,r'\d{4}-\d\d-\d\dT\d\d:\d\d:')
            self.assertNotIn('source_xml"',text)
            self.assertNotIn('CommandLine',text)
            self.assertNotIn('ProcessId',text)

    def test_receiver_integration_never_claims_push_recovery_pass(self):
        data=json.loads((ROOT/'reports/standing-system-audit-20261020/receiver-integration-summary.json').read_text('utf-8'))
        self.assertTrue(data['compiled_model_arrays_identical'])
        self.assertTrue(data['nominal_stand_20s_passed'])
        self.assertEqual(data['pushed_recovery_cases_passed_after'],0)
        self.assertTrue(data['negative_direction_terminal_tilt_worsened'])
        self.assertEqual(data['hardware_operations'],0)

if __name__=='__main__':unittest.main(verbosity=2)
