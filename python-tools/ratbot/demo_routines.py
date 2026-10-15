"""Nonblocking command playback for the demo; never used by physical control."""
from collections import deque
from routine_generator import greet


class CommandSequence:
    def __init__(self):
        self.elapsed = 0.0
        self.commands = []

    def move_all(self, positions, dur=0, record=False):
        self.commands.append((self.elapsed, list(positions), dur))
        return True

    def wait(self, seconds):
        self.elapsed += seconds


def greeting_sequence(rest):
    sequence = CommandSequence()
    greet(sequence, wait=sequence.wait)
    sequence.move_all(rest, 1000)
    sequence.wait(1)
    return sequence


def jump_sequence(rest=None):
    """Mirror q8Dynamixel::jump() and its header's angle pairs.

    Profile zero supplies no timed servo path: display those targets immediately.
    This reproduces joint commands, not takeoff, flight or servo acceleration.
    """
    sequence = CommandSequence()
    sequence.wait(.1)  # setProfile(500); delay(100)
    sequence.move_all([-40, 220] * 4, 500)
    sequence.wait(1.1)  # delay(1000); setProfile(0); setGain(800); delay(100)
    sequence.move_all([90, 90] * 4, 0)
    sequence.wait(.1)
    sequence.move_all([30, 150] * 4, 0)
    sequence.wait(5.1)  # delay(5000); setProfile(500); delay(100)
    sequence.move_all([30, 150] * 4, 500)  # _idlePos
    sequence.wait(1)
    if rest is not None:
        # Wait for the full firmware sequence before the GUI's gait-rest command.
        sequence.move_all(rest, 500)
        sequence.wait(.5)
    return sequence


class DemoRoutine:
    def __init__(self, name, sequence, controller, clock):
        self.name = name
        self.controller = controller
        self.clock = clock
        self.started = clock()
        self.ends = self.started + sequence.elapsed
        self.commands = deque(sequence.commands)

    @property
    def active(self):
        return bool(self.commands) or self.clock() < self.ends

    def update(self):
        now = self.clock()
        while self.commands and self.started + self.commands[0][0] <= now:
            offset, positions, duration = self.commands.popleft()
            self.controller.move_all(positions, duration, False, command_time=self.started + offset)
