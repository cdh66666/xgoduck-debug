"""Unloaded manufacturer calibration for the vendor fixed-2047 controller."""
from pathlib import Path
from factory_calibration import FactoryCalibrationMixin, atomic_json


class NativeFactory(FactoryCalibrationMixin):
    def __init__(self, controller, root):
        self.controller=controller
        self.bridge=controller.bridge
        self.lock=controller.lock
        self.servo_op_lock=controller.servo_op_lock
        self.zero_pos=[2047]*15
        self._factory_init(Path(root)/'data/native-calibration/zero_pos.json')

    @property
    def mode(self):return self.controller.mode

    @property
    def feedback(self):return self.controller.feedback

    @property
    def web_time(self):return self.controller.web_time

    @web_time.setter
    def web_time(self, value):self.controller.web_time=value

    def set_mode(self, mode):return self.controller.set_mode(mode)

    def _save_zeros(self):
        # The vendor MCU reports calibrated joints about its fixed center 2047.
        # It returns this reference, separately from encoder position 2048.
        if self.zero_pos != [2047]*15:raise ValueError('Unexpected vendor MCU reference')
        atomic_json(self.factory_zero_path,self.zero_pos)

    def factory_action(self, action, target_id=10):
        if action=='calibrate':action='calibrate_unloaded'
        if action=='calibrate_unloaded' and not self.factory_caps & 0x40:
            raise ValueError('Calibration paused: direct-offset unloaded MCU support required')
        return super().factory_action(action,target_id)

    def factory_status(self):
        with self.lock:
            return dict(super().factory_status(),calibration_loads_motor=False,
                        unloaded_calibration_supported=bool(self.factory_caps & 0x40),
                        coordinate_reference='vendor fixed encoder center 2047')
