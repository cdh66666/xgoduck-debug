"""Actual authenticated encryption and isolated import; no controller access."""
import importlib.util,io,json,tempfile,unittest,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('connection_import',ROOT/'pc/tools/import_connection.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
PASSWORD='fixture-password-not-a-real-device-key'
def entries():
 return {'device.local.json':b'{"serial":"TEST_BOARD"}',
 'tools/.private/ssh-identity.json':b'{"serial":"TEST_BOARD","key_path":"id_ed25519","known_hosts_path":"known_hosts"}',
 'tools/.private/id_ed25519':b'fake-key-bytes',
 'tools/.private/known_hosts':b'fake-host-pinning',
 'tools/.private/wireless-calibration-auth.json':b'{"key":"fixture-only"}',
 'qa/wireless-connection.json':b'{"network_url":"http://192.168.1.50:9527"}'}
def archive(files):
 out=io.BytesIO()
 with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
  for name,raw in files.items():z.writestr(name,raw)
 return out.getvalue()
class ImportTests(unittest.TestCase):
 def test_roundtrip_exact_files_and_no_robot_action(self):
  values=entries();blob=module.encrypt_bundle(archive(values),PASSWORD)
  self.assertNotIn(b'fake-key-bytes',blob)
  with tempfile.TemporaryDirectory() as folder:
   result=module.install_bundle(blob,PASSWORD,folder,permissions=False)
   self.assertEqual(result['network_requests'],0);self.assertEqual(result['robot_actions'],0)
   for name,raw in values.items():self.assertEqual((Path(folder)/name).read_bytes(),raw)
 def test_wrong_password_writes_nothing(self):
  blob=module.encrypt_bundle(archive(entries()),PASSWORD)
  with tempfile.TemporaryDirectory() as folder:
   with self.assertRaisesRegex(ValueError,'口令'):module.install_bundle(blob,'wrong-password',folder,permissions=False)
   self.assertEqual(list(Path(folder).iterdir()),[])
 def test_tampered_ciphertext_is_rejected_before_writes(self):
  blob=bytearray(module.encrypt_bundle(archive(entries()),PASSWORD));blob[-1]^=1
  with tempfile.TemporaryDirectory() as folder:
   with self.assertRaises(ValueError):module.install_bundle(bytes(blob),PASSWORD,folder,permissions=False)
   self.assertEqual(list(Path(folder).iterdir()),[])
 def test_authentication_protects_header(self):
  blob=bytearray(module.encrypt_bundle(archive(entries()),PASSWORD));blob[10]^=1
  with self.assertRaises(ValueError):module.decrypt_bundle(bytes(blob),PASSWORD)
 def test_path_escape_or_extra_files_rejected(self):
  bad=entries();bad['../escape']=b'no'
  with tempfile.TemporaryDirectory() as folder:
   with self.assertRaisesRegex(ValueError,'文件范围'):module.install_bundle(module.encrypt_bundle(archive(bad),PASSWORD),PASSWORD,folder,permissions=False)
   self.assertEqual(list(Path(folder).iterdir()),[])
 def test_existing_different_configuration_not_overwritten(self):
  with tempfile.TemporaryDirectory() as folder:
   path=Path(folder)/'device.local.json';path.write_bytes(b'other-config')
   with self.assertRaisesRegex(ValueError,'未覆盖'):module.install_bundle(module.encrypt_bundle(archive(entries()),PASSWORD),PASSWORD,folder,permissions=False)
   self.assertEqual(path.read_bytes(),b'other-config');self.assertEqual(len(list(Path(folder).iterdir())),1)
 def test_new_salt_and_nonce_each_time(self):
  raw=archive(entries());self.assertNotEqual(module.encrypt_bundle(raw,PASSWORD),module.encrypt_bundle(raw,PASSWORD))
if __name__=='__main__':unittest.main()
