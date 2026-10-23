import importlib.util,json,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('plain_import',ROOT/'pc/tools/import_connection.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
def fixture(root):
 p=Path(root)/'profile';p.mkdir()
 data={'device.json':b'{"serial":"TEST_BOARD"}',
 'ssh-identity.json':b'{"serial":"TEST_BOARD","key_path":"id_ed25519","known_hosts_path":"known_hosts"}',
 'id_ed25519':b'fixture-key','known_hosts':b'fixture-pin',
 'wireless-auth.json':b'{"key":"fixture-auth"}','network.json':b'{"network_url":"http://192.168.1.50:9527"}'}
 for name,raw in data.items():(p/name).write_bytes(raw)
 return p,data
class ProfileTests(unittest.TestCase):
 def test_exact_copy_without_password_or_network(self):
  with tempfile.TemporaryDirectory() as folder:
   p,data=fixture(folder);out=Path(folder)/'pc';v=module.install_profile(p,out,permissions=False)
   self.assertEqual(v['robot_actions'],0);self.assertEqual(v['network_requests'],0)
   for name,raw in data.items():self.assertEqual((out/module.MAPPING[name]).read_bytes(),raw)
   self.assertEqual(json.loads((out/'tools/.private/connection-import.json').read_text())['files'],6)
 def test_missing_file_does_not_install_partial_profile(self):
  with tempfile.TemporaryDirectory() as folder:
   p,_=fixture(folder);(p/'known_hosts').unlink();out=Path(folder)/'pc'
   with self.assertRaises(ValueError):module.install_profile(p,out,permissions=False)
   self.assertFalse(out.exists())
 def test_serial_mismatch_rejected_before_writes(self):
  with tempfile.TemporaryDirectory() as folder:
   p,_=fixture(folder);(p/'device.json').write_text('{"serial":"OTHER"}');out=Path(folder)/'pc'
   with self.assertRaises(ValueError):module.install_profile(p,out,permissions=False)
   self.assertFalse(out.exists())
 def test_existing_different_config_preserved(self):
  with tempfile.TemporaryDirectory() as folder:
   p,_=fixture(folder);out=Path(folder)/'pc';out.mkdir();(out/'device.local.json').write_bytes(b'other')
   with self.assertRaises(ValueError):module.install_profile(p,out,permissions=False)
   self.assertEqual((out/'device.local.json').read_bytes(),b'other')
 def test_same_profile_can_be_imported_again(self):
  with tempfile.TemporaryDirectory() as folder:
   p,_=fixture(folder);out=Path(folder)/'pc';module.install_profile(p,out,permissions=False)
   self.assertEqual(module.install_profile(p,out,permissions=False)['files'],6)
 def test_explicit_replace_uses_selected_device(self):
  with tempfile.TemporaryDirectory() as folder:
   p,data=fixture(folder);out=Path(folder)/'pc';out.mkdir();(out/'device.local.json').write_bytes(b'other')
   module.install_profile(p,out,replace=True,permissions=False)
   self.assertEqual((out/'device.local.json').read_bytes(),data['device.json'])
if __name__=='__main__':unittest.main()
