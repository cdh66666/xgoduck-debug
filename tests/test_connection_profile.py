import importlib.util,json,os,shutil,subprocess,tempfile,unittest
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
 @unittest.skipUnless(os.name=='nt' and shutil.which('ssh-keygen'),'Windows OpenSSH required')
 def test_windows_openssh_accepts_key_after_owner_rights_removed(self):
  with tempfile.TemporaryDirectory() as folder:
   key=Path(folder)/'id_ed25519'
   subprocess.run(['ssh-keygen','-q','-t','ed25519','-N','','-f',str(key)],check=True,capture_output=True)
   subprocess.run(['icacls',str(key),'/grant','*S-1-3-4:F'],check=True,capture_output=True)
   before=subprocess.run(['ssh-keygen','-y','-P','','-f',str(key)],capture_output=True)
   self.assertNotEqual(before.returncode,0)
   self.assertIn(b'permissions',before.stderr.lower())
   module._restrict_private_files([key])
   after=subprocess.run(['ssh-keygen','-y','-P','','-f',str(key)],capture_output=True)
   self.assertEqual(after.returncode,0,after.stderr.decode('utf-8',errors='replace'))
   self.assertTrue(after.stdout.startswith(b'ssh-ed25519 '))
if __name__=='__main__':unittest.main()
