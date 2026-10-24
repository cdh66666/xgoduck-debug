"""OFF-only stationary qualification; a correction freezes at explicit start.

No device IO, no calibration writes, no adaptive estimator during policy motion.
The MCU display epoch identifies the boot; display calibration READY is not used
as acceptance evidence. Quality is recomputed from the raw control feedback.
"""
import collections
import math


class FrozenGyroBias:
    MIN_SAMPLES = 201
    MIN_SECONDS = 2.0
    MAX_SOURCE_GAP_US = 20000
    MAX_SAMPLE_AGE_S = .15
    MAX_GYRO_STD_DPS = .35
    MAX_GYRO_MEAN_DPS = 20.
    GRAVITY_DOT_MIN = math.cos(math.radians(2.))
    MAX_JOINT_SPEED_DPS = 2.
    MAX_JOINT_SPAN_DEG = .35
    ALL_JOINTS_MASK = 0x7fff

    def __init__(self):
        self.reset()

    def reset(self, epoch=None, reason='not_qualified'):
        self.epoch = epoch
        self.frozen = None
        self.samples = collections.deque(maxlen=256)
        self.last_seq = self.last_us = self.last_received = None
        self.ready = False
        self.reason = reason
        self.mean = self.std = None
        self.duration_s = 0.
        self.application = None
        self._clear_window(reason)

    def _clear_window(self, reason):
        self.samples.clear()
        self._gyro_sum = [0.] * 3
        self._gyro_sum_sq = [0.] * 3
        self._joint_index = 0
        self._joint_min = [collections.deque() for _ in range(15)]
        self._joint_max = [collections.deque() for _ in range(15)]
        self.ready = False
        self.reason = reason
        self.mean = self.std = None
        self.duration_s = 0.

    def _track_joint_window(self, angles):
        # Amortized O(15): rescanning 15*256 angles at 100 Hz stalls the receiver
        # on the board. Monotonic queues retain the exact rolling extrema.
        index=self._joint_index;self._joint_index+=1
        cutoff=index-self.samples.maxlen+1
        quiet=True
        for i,value in enumerate(angles):
            lo,hi=self._joint_min[i],self._joint_max[i]
            while lo and lo[0][0]<cutoff:lo.popleft()
            while hi and hi[0][0]<cutoff:hi.popleft()
            while lo and lo[-1][1]>=value:lo.pop()
            while hi and hi[-1][1]<=value:hi.pop()
            lo.append((index,value));hi.append((index,value))
            if hi[0][1]-lo[0][1]>self.MAX_JOINT_SPAN_DEG:quiet=False
        return quiet

    def observe(self, *, acc, gyro, angles, velocities, seq, sample_us,
                mask, imu_ok, eligible, epoch, now):
        raw_valid = (len(acc) == 3 and len(gyro) == 3 and imu_ok
                     and all(math.isfinite(v) for v in [*acc, *gyro, now]))
        if self.frozen is not None:
            # Caller disarms before clearing a frozen correction; no live adaptation.
            if epoch and epoch != self.epoch:
                return 'epoch_changed'
            if not raw_valid:
                return 'imu_invalidated'
            self.last_seq, self.last_us, self.last_received = seq, sample_us, now
            return 'locked'
        if not epoch:
            self._clear_window('waiting_for_fresh_mcu_epoch')
            return 'waiting'
        if self.epoch != epoch:
            self.reset(epoch, 'new_mcu_epoch')
        if not eligible:
            self._clear_window('requires_explicit_off')
            self.last_seq = self.last_us = None
            return 'waiting'
        if self.last_seq is not None:
            delta = (seq-self.last_seq) & 0xffffffff
            if delta == 0:
                return 'duplicate_ignored'
            gap = (sample_us-self.last_us) & 0xffffffff
            if delta != 1 or not 0 < gap <= self.MAX_SOURCE_GAP_US:
                self._clear_window('source_gap_or_restart')
        self.last_seq, self.last_us, self.last_received = seq, sample_us, now
        if not raw_valid:
            self._clear_window('invalid_imu_feedback')
            return 'rejected'
        if (mask != self.ALL_JOINTS_MASK or len(angles) != 15 or len(velocities) != 15
                or not all(math.isfinite(v) for v in [*angles, *velocities])):
            self._clear_window('incomplete_joint_feedback')
            return 'rejected'
        if max(abs(v) for v in velocities) > self.MAX_JOINT_SPEED_DPS:
            self._clear_window('joints_moving')
            return 'rejected'
        norm = math.sqrt(sum(v*v for v in acc))
        if abs(norm/9.8-1) > .1 or norm < 1e-6:
            self._clear_window('acceleration_not_stationary')
            return 'rejected'
        gravity = tuple(v/norm for v in acc)
        if max(abs(v) for v in gyro) > self.MAX_GYRO_MEAN_DPS:
            self._clear_window('motion_or_excessive_gyro')
            return 'rejected'
        if self.samples:
            first = self.samples[0]
            if sum(a*b for a,b in zip(gravity,first[2])) < self.GRAVITY_DOT_MIN:
                self._clear_window('pose_changed')
        if not self._track_joint_window(angles):
            self._clear_window('joint_pose_changed')
            self._track_joint_window(angles)
        # Keep the exact same finite window without rescanning 3*256 samples
        # twice on every 100 Hz callback while holding the controller lock.
        # Raw DQS1 gyro units are 1/32 dps; these bounded sums are exact doubles.
        if len(self.samples) == self.samples.maxlen:
            expired = self.samples[0][1]
            for i, value in enumerate(expired):
                self._gyro_sum[i] -= value
                self._gyro_sum_sq[i] -= value*value
        for i, value in enumerate(gyro):
            self._gyro_sum[i] += value
            self._gyro_sum_sq[i] += value*value
        self.samples.append((sample_us, tuple(gyro), gravity, tuple(angles)))
        self.duration_s = ((sample_us-self.samples[0][0]) & 0xffffffff)/1e6
        if len(self.samples) < self.MIN_SAMPLES or self.duration_s < self.MIN_SECONDS:
            self.reason = 'collecting_off_stationary_window'
            return 'collecting'
        n = len(self.samples)
        mean = tuple(value/n for value in self._gyro_sum)
        std = tuple(math.sqrt(max(0., self._gyro_sum_sq[i]-self._gyro_sum[i]*mean[i])/(n-1)) for i in range(3))
        self.mean, self.std = mean, std
        self.ready = max(std) <= self.MAX_GYRO_STD_DPS
        self.reason = 'qualified_pending_explicit_start' if self.ready else 'gyro_variance_too_high'
        return 'ready' if self.ready else 'rejected'

    def freeze(self, *, epoch, now):
        if self.frozen is not None:
            if epoch and epoch != self.epoch:
                raise ValueError('MCU epoch changed; OFF stationary qualification required')
            return self.frozen
        if (not self.ready or epoch != self.epoch or self.last_received is None
                or not 0 <= now-self.last_received <= self.MAX_SAMPLE_AGE_S):
            raise ValueError('OFF stationary gyro qualification requires >=2s fresh quiet feedback')
        self.frozen = tuple(self.mean)
        self.reason = 'locked_until_mcu_or_host_restart'
        return self.frozen

    def correct(self, gyro):
        return [v-b for v,b in zip(gyro,self.frozen)] if self.frozen is not None else list(gyro)

    def mark_applied(self, *, seq, sample_us, sample_clock_s, now, raw_acc,
                     corrected_gyro_dps, before_gravity, after_gravity):
        if self.frozen is None or self.application is not None:
            raise ValueError('gyro correction application must be one explicit-start event')
        self.application = dict(applied_at_mcu_seq=seq, applied_at_sample_us=sample_us,
            applied_at_sample_clock_s=sample_clock_s, applied_at_host_monotonic_s=now,
            gravity_seed='normalized_current_OFF_stationary_acc', raw_acc=list(raw_acc),
            corrected_gyro_dps=list(corrected_gyro_dps), before_gravity=list(before_gravity),
            after_gravity=list(after_gravity), filter_reset=True)

    def status(self):
        return dict(qualified=self.ready, correction_active=self.frozen is not None,
                    frozen_bias_dps=list(self.frozen) if self.frozen is not None else None,
                    candidate_bias_dps=list(self.mean) if self.mean else None,
                    gyro_std_dps=list(self.std) if self.std else None,
                    samples=len(self.samples), duration_s=self.duration_s,
                    boot_epoch=self.epoch, reason=self.reason,
                    applied=self.application is not None,
                    applied_at_mcu_seq=self.application['applied_at_mcu_seq'] if self.application else None,
                    application=dict(self.application) if self.application else None,
                    source='independent_OFF_raw_feedback_window', display_ready_used=False)
