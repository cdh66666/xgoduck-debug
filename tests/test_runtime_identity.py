import hashlib,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot-app/python'))
from runtime_identity import RuntimeIdentity
from servo_health import PACKET,ServoHealth

class RuntimeIdentityTests(unittest.TestCase):
    def test_health_packet_matches_packed_mcu_layout(self):
        h=ServoHealth();raw=PACKET.pack(b'DQH2',100,1200,2,3,4,*([100]*15),*([0]*15),*([0]*15),32766,b'\xff'*32,32,b'\x01'*16,16)+b'20261025-feedback-r1'.ljust(24,b'\0')
        h.receive(raw);v=h.status();self.assertEqual(v['build'],'20261025-feedback-r1');self.assertEqual(v['ids']['10']['valid_replies'],100)
        self.assertEqual(v['last_incomplete_mask'],32766);h.receive(b'bad');self.assertEqual(h.status()['invalid_packets'],1)
    def test_disk_file_is_not_claimed_as_running_without_live_mcu(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'python').mkdir();(root/'.cache/sketch').mkdir(parents=True);(root/'data').mkdir()
            file=root/'python/controller.py';file.write_text('v1');elf=root/'.cache/sketch/sketch.ino.elf';elf.write_bytes(b'firmware')
            sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
            (root/'data/firmware-release.json').write_text(json.dumps({'version':'v1','mcu_build':'v1','python_sha256':{'python/controller.py':sha(file)},'elf_sha256':sha(elf)}))
            class Health:
                value={'build':'old','age_s':0}
                def status(self):return self.value
            h=Health();identity=RuntimeIdentity(root,h);self.assertFalse(identity.snapshot()['running_verified'])
            h.value={'build':'v1','age_s':0};self.assertTrue(identity.snapshot()['running_verified'])
            file.write_text('new on disk');self.assertNotEqual(identity.snapshot()['python_sha256']['python/controller.py'],sha(file))
            h.value['age_s']=4;self.assertFalse(identity.snapshot()['running_verified'])

if __name__=='__main__':unittest.main()
