"""Recognize the user's neutral calibration-tray pose from body-frame feedback."""
import math

TRAY_REASON = '检测到中位校准托盘姿态：禁止上力和运动'
JOINT_TOLERANCE_DEG = 8.0
BODY_TOLERANCE_DEG = 20.0
MIN_NEUTRAL_JOINTS = 12


def is_tray_pose(feedback):
    # The measured fixture pose is near zero at every joint, lying on body -X.
    # Upright zero joints and a folded robot on the floor are different poses.
    if feedback is None or not feedback.imu_ok:
        return False
    if len(feedback.angles) != 15 or len(feedback.acc) != 3:
        return False
    values = (*feedback.angles, *feedback.acc)
    if not all(math.isfinite(v) for v in values):
        return False
    norm = math.sqrt(sum(v*v for v in feedback.acc))
    return (norm > 1.0
            and -feedback.acc[0]/norm >= math.cos(math.radians(BODY_TOLERANCE_DEG))
            and sum(bool(feedback.mask & (1 << i)) and abs(v) <= JOINT_TOLERANCE_DEG
                    for i,v in enumerate(feedback.angles)) >= MIN_NEUTRAL_JOINTS)


def require_motion_clear(feedback):
    if is_tray_pose(feedback):
        raise ValueError(TRAY_REASON)
