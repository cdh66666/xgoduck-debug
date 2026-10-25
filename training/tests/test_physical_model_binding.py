import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import native_env
from training_adapter import Adapter

ROOT=Path(__file__).resolve().parents[1]
CANDIDATE=ROOT/'model/robot_inertial_candidate_20261025.xml'


class PhysicalModelBindingTests(unittest.TestCase):
    def test_selected_candidate_is_actually_loaded_with_same_hash(self):
        with patch.dict(os.environ,{'XGODUCK_TRAIN_XML':str(CANDIDATE)}):
            frozen=native_env.manifest()
        env=native_env.NativeEpisode(frozen,20261025,curriculum='stand')
        try:
            self.assertEqual(env.sim.xml_sha,frozen['physical_model']['sha256'])
            self.assertNotEqual(env.sim.xml_sha,frozen['physical_model']['reference_model_sha256'])
            self.assertAlmostEqual(float(env.sim.model.body_mass.sum()),.826)
            self.assertEqual(env.sim.extra_contacts,13)
        finally:env.close()

    def test_changed_selected_file_is_rejected_before_worker_load(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'robot.xml';path.write_bytes(CANDIDATE.read_bytes())
            with patch.dict(os.environ,{'XGODUCK_TRAIN_XML':str(path)}):frozen=native_env.manifest()
            path.write_bytes(path.read_bytes()+b'\n')
            with self.assertRaisesRegex(ValueError,'changed before worker load'):
                Adapter(frozen,20261025,'test')


if __name__=='__main__':unittest.main()
