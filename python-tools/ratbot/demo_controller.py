"""In-memory implementation of the controller interface used by operate.py.

No transport is created. Motion targets are retained for inspection, while
hardware-only actions succeed without doing anything or fabricating telemetry.
"""

import time


class _EmptyReplies:
    """Keep the GUI's reply-draining loop compatible without a serial port."""

    in_waiting = 0

    def readline(self):
        return b''


class DemoController:
    def __init__(self, clock=time.monotonic):
        self.serialHandler = _EmptyReplies()
        self.torque_on = False
        self.last_positions = None
        self.last_duration = 0
        self.recording = False
        self.command_frames = 0
        self._clock = clock
        self._start_positions = None
        self._started_at = 0.0
        self.routine = None

    @property
    def routine_active(self):
        return self.routine is not None and self.routine.active

    def update_routine(self):
        if self.routine is not None:
            self.routine.update()

    def start_greeting(self, rest):
        from demo_routines import DemoRoutine, greeting_sequence
        self.routine = DemoRoutine('GREETING', greeting_sequence(rest), self, self._clock)
        self.update_routine()

    def start_jump(self, rest=None):
        from demo_routines import DemoRoutine, jump_sequence
        self.routine = DemoRoutine('JUMP', jump_sequence(rest), self, self._clock)

    def enable_torque(self):
        self.torque_on = True
        return True

    def disable_torque(self):
        self.torque_on = False
        return True

    def move_all(self, joints_pos, dur=0, record=True, *, command_time=None):
        now = self._clock() if command_time is None else command_time
        self._start_positions = self.sample_positions(now)
        self.last_positions = list(joints_pos)
        self._started_at = now
        self.last_duration = dur
        self.recording = bool(record)
        self.command_frames += 1
        return True

    def sample_positions(self, now=None):
        """Visual interpolation of commanded durations, never motor feedback.

        Zero-duration gait frames are exact. The first command establishes the
        initial configuration; interrupted transitions start at the current sample.
        """
        if self.last_positions is None:
            return None
        if self._start_positions is None or self.last_duration <= 0:
            return list(self.last_positions)
        elapsed = (self._clock() if now is None else now) - self._started_at
        fraction = max(0.0, min(1.0, elapsed / (self.last_duration / 1000)))
        if fraction == 1:
            return list(self.last_positions)
        return [start + (target - start) * fraction
                for start, target in zip(self._start_positions, self.last_positions)]

    def move_mirror(self, joint_pos, dur=0):
        return self.move_all(list(joint_pos) * 4, dur, False)

    def check_battery(self):
        # There is no battery telemetry in a hardware-free demo.
        return True

    def record_data(self):
        self.recording = True
        return True

    def finish_recording(self):
        self.recording = False
        return True

    def send_jump(self):
        self.start_jump()
        return True

    def send_recover(self):
        return self.enable_torque()
