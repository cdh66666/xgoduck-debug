import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'robot-app/python'))
from factory_calibration import FactoryCalibrationMixin
from wire import FACTORY,FACTORY_REPLY,decode_factory_reply


class UnloadedFactoryTests(unittest.TestCase):
    def test_calibration_in_tray_uses_unloaded_opcode_and_keeps_journal(self):
        with tempfile.TemporaryDirectory() as folder:
            c=FactoryCalibrationMixin();c.lock=threading.RLock();c.servo_op_lock=threading.Lock()
            c._factory_init(Path(folder)/'zero.json');c.factory_caps=127
            c.mode='servo_debug';c.factory_active=True;c.zero_pos=[2047]*15
            c.feedback=SimpleNamespace(imu_ok=True,mask=0x7fff,acc=(-9.8,0,0),angles=[0.]*15)
            calls=[];saved=[];c._save_zeros=lambda:saved.append(list(c.zero_pos))
            def notify(name,packet):
                magic,op,target,reserved,request=FACTORY.unpack(packet);calls.append(op)
                if op==5:
                    pending=json.loads((Path(folder)/'factory_calibration/pending.json').read_text())
                    self.assertEqual(pending['before']['position'],3600)
                pos=2048 if op==5 else 3600
                raw=FACTORY_REPLY.pack(b'DQF2',op,target,0,int(op==5),request,pos,pos,123,pos,4,0,1,0,0,(1<<8) if op==5 else 0,127,0)
                c.receive_factory_reply(raw)
            c.bridge=SimpleNamespace(notify=notify)
            result=c.factory_action('calibrate',23)
            self.assertEqual(calls,[1,5]);self.assertEqual(result['torque'],0)
            self.assertEqual(result['locked'],1);self.assertEqual(result['held_mask'],0)
            self.assertEqual(len(saved),1);self.assertTrue(result['host_zero_saved'])
            self.assertFalse((Path(folder)/'factory_calibration/pending.json').exists())

    def test_old_firmware_cannot_receive_unloaded_opcode(self):
        for caps in (31,63):
            c=FactoryCalibrationMixin();c.servo_op_lock=threading.Lock();c.mode='servo_debug';c.factory_active=True;c.factory_caps=caps
            with self.assertRaisesRegex(ValueError,'does not support unloaded'):
                c.factory_action('calibrate_unloaded',23)

    def test_vendor_adapter_blocks_the_failed_sdk_prototype(self):
        from native_factory import NativeFactory
        c=NativeFactory.__new__(NativeFactory);c.factory_caps=63
        with self.assertRaisesRegex(ValueError,'Calibration paused'):
            c.factory_action('calibrate',23)


if __name__=='__main__':unittest.main()
