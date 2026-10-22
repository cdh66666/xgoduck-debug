"""One walk actor for every admitted keyboard point; Controller owns output.

Legacy specialist flags remain request metadata. They never select/load another
actor, copy history, delay a release, or reset the walk action filter.
"""
from __future__ import annotations

import math

WALK_FILE = 'xgoduck_walk.onnx'
WALK_SHA256 = '8a462e51d41fd43cbca1026698a2320f76fd84ef61d5c081cc63b7bcad7e394f'
KEYBOARD_POINTS = ((0., 0., 0.), (.2, 0., 0.), (.3, 0., 0.),
                   (-.3, 0., 0.), (0., 0., 1.), (0., 0., -1.))


class KeyboardRuntimeMixin:
    def _keyboard_init(self):
        self._keyboard_cancel_locked()
        self.model_message += '; unified keyboard walk actor, zero head/body commands'

    def _keyboard_cancel_locked(self):
        self.twist, self.head = [0.]*3, [0.]*4
        self.manual_pick_phase = self.manual_pick_target = 0.0
        self.manual_pick_returning = False
        self.fast_requested = self.fast_shift_requested = False
        self.navigation_requested = False
        # Compatibility state for the retained automatic-action guards only.
        self.fast_active = self.navigation_active = False
        self._keyboard_manual_blending = False
        self._keyboard_recovery_latched = False

    def _keyboard_recovery_command_locked(self, twist, head, depth, forward, fast, navigation=False):
        if self.mode != 'policy':
            return False
        if self.recovery.phase != 'walk':
            self._keyboard_cancel_locked()
            self._keyboard_recovery_latched = True
            self.twist, self.head = [0.]*3, [0.]*4
            return True
        if self._keyboard_recovery_latched:
            neutral = (twist is not None and all(abs(v) < 1e-9 for v in twist)
                       and (head is None or all(abs(v) < 1e-9 for v in head))
                       and (depth is None or type(depth) in (int, float) and depth == 0)
                       and forward in (None, False) and fast is False and navigation is False)
            self.twist, self.head = [0.]*3, [0.]*4
            if neutral:
                self._keyboard_recovery_latched = False
            # The acknowledgement stays neutral; a later explicit command moves.
            return True
        return False

    def _keyboard_command_locked(self, twist, depth, fast, forward=None, navigation=False):
        if type(fast) is not bool or type(navigation) is not bool or (forward is not None and type(forward) is not bool):
            raise ValueError('keyboard compatibility flags must be boolean')
        if depth is not None:
            if isinstance(depth, bool) or not math.isfinite(float(depth)) or float(depth) != 0:
                raise ValueError('deep Pick is not admitted by the unified keyboard model')
        if twist is not None and not any(all(abs(a-b) < 1e-7 for a,b in zip(twist, point)) for point in KEYBOARD_POINTS):
            raise ValueError('unified keyboard admits only zero, forward .2/.3, reverse -.3, yaw +/-1')
        self.fast_requested = bool(forward)
        self.fast_shift_requested = fast
        self.navigation_requested = navigation

    def _keyboard_select_locked(self, selected, twist, head, mouth, *, mode, phase, valid, now, automatic):
        self._keyboard_manual_blending = False
        if mode != 'policy' or phase != 'walk':
            self._keyboard_cancel_locked()
            if mode == 'policy' and phase != 'walk':
                self._keyboard_recovery_latched = True
            return selected, twist, [0.]*4, mouth
        if self._keyboard_recovery_latched:
            return selected, [0.]*3, [0.]*4, mouth
        if automatic:
            # Published entry guards reject automatic actions. Preserve their
            # original selector should an offline fixture exercise that path.
            return selected, twist, [0.]*4, mouth
        return self.policy, twist, [0.]*4, mouth

    def _keyboard_state_locked(self, now):
        common = dict(available=self.policy.loaded, active=False, braking=False,
                      brake_remaining_s=0., model=WALK_FILE, artifact_sha256=WALK_SHA256,
                      source='unified_walk_actor', allowed_points=[list(p) for p in KEYBOARD_POINTS],
                      head_deferred=False)
        return dict(
            motion_recovery_latched=self._keyboard_recovery_latched,
            keyboard_policy=dict(model=WALK_FILE, artifact_sha256=WALK_SHA256,
                                 source='unified_walk_actor', history_continuous=True,
                                 head_command=[0.]*4, body_command=[0.]*6),
            manual_pick=dict(active=False, depth=0., target_depth=0., phase=0.,
                             returning=False, available=False, mouth_owner='user'),
            forward_specialist=dict(common, requested=self.fast_requested,
                                    accelerated=self.fast_shift_requested, speed=self.twist[0]),
            fast_forward=dict(common, requested=self.fast_shift_requested,
                              accelerated=self.fast_shift_requested, speed=self.twist[0]),
            navigation_specialist=dict(common, requested=self.navigation_requested,
                                      command=list(self.twist)))
