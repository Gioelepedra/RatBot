"""In-memory implementation of the controller interface used by operate.py.

No transport is created. Motion targets are retained for inspection, while
hardware-only actions succeed without doing anything or fabricating telemetry.
"""


class _EmptyReplies:
    """Keep the GUI's reply-draining loop compatible without a serial port."""

    in_waiting = 0

    def readline(self):
        return b''


class DemoController:
    def __init__(self):
        self.serialHandler = _EmptyReplies()
        self.torque_on = False
        self.last_positions = None
        self.last_duration = 0
        self.recording = False
        self.command_frames = 0

    def enable_torque(self):
        self.torque_on = True
        return True

    def disable_torque(self):
        self.torque_on = False
        return True

    def move_all(self, joints_pos, dur=0, record=True):
        self.last_positions = list(joints_pos)
        self.last_duration = dur
        self.recording = bool(record)
        self.command_frames += 1
        return True

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
        return True

    def send_recover(self):
        return self.enable_torque()
