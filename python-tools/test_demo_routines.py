"""Demo contact constraints and exact nonblocking command schedules."""
import math
from pathlib import Path
import re
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'ratbot'))
from body_pose import support_transform
from demo_controller import DemoController
from demo_routines import CommandSequence, greeting_sequence, jump_sequence
from kinematic_preview import KinematicPreview
from kinematics_solver import k_solver
from leg_geometry import Q8FiveBarGeometry
from rearing_pose import RearingPose
from routine_generator import greet, G1, G2, G3, G4, G5, G6


class DemoRoutineTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.robot = DemoController(clock=lambda: self.now)
        self.rest = [39.4, 140.6] * 4
        self.robot.move_all(self.rest, 0, False)

    def test_greeting_preserves_hardware_calls_and_all_waits(self):
        # Record the real/default path: its targets, durations and sleeps must match.
        hardware = CommandSequence()
        with patch('routine_generator.time.sleep', side_effect=hardware.wait):
            greet(hardware)
        expected = greeting_sequence(self.rest)
        self.assertEqual(hardware.commands, expected.commands[:-1])
        self.assertEqual([c[1] for c in hardware.commands],
                         [G1, G2, G3, G4, G5, G4, G5, G4, G5, G6])
        for actual, seconds in zip([c[0] for c in hardware.commands],
                                   [0, 1.1, 2.1, 3.1, 3.35, 3.6, 3.85, 4.1, 4.35, 5.35]):
            self.assertAlmostEqual(actual, seconds)
        self.assertAlmostEqual(hardware.elapsed, 6.05)
        with patch('time.sleep', side_effect=AssertionError('Demo must not sleep')):
            self.robot.start_greeting(self.rest)
            seen = []
            previous_count = 1
            for step in range(707):
                self.now = step / 100
                self.robot.update_routine()
                if self.robot.command_frames != previous_count:
                    seen.append(self.robot.last_positions[:])
                    previous_count = self.robot.command_frames
            self.assertEqual(seen, [c[1] for c in expected.commands])
            self.assertFalse(self.robot.routine_active)
            self.assertEqual(self.robot.sample_positions(), self.rest)

    def test_jump_schedule_matches_current_firmware_source(self):
        base = ROOT.parent / 'firmware' / 'ratbot_robot'
        header = (base / 'include' / 'q8Dynamixel.h').read_text()
        source = (base / 'src' / 'q8Dynamixel.cpp').read_text()
        body = source.split('void q8Dynamixel::jump(){', 1)[1].split('\n}', 1)[0]
        pairs = {array: [float(v) for v in re.search(rf'{angle}\[2\]\s*=\s*\{{([^}}]+)', header)[1].split(',')] * 4
                 for array, angle in (('_lowArray', '_jumpLow'), ('_highArray', '_jumpHigh'),
                                      ('_restArray', '_jumpRest'), ('_idleArray', '_idlePos'))}
        commands, elapsed, duration = [], 0.0, None
        for action, value in re.findall(r'(setProfile|delay|bulkWrite)\(([^)]+)\)', body):
            if action == 'setProfile':
                duration = int(value)
            elif action == 'delay':
                elapsed += int(value) / 1000
            else:
                commands.append((elapsed, pairs[value], duration))
        sequence = jump_sequence()
        self.assertEqual(len(sequence.commands), 4)
        for actual, expected in zip(sequence.commands, commands):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertEqual(actual[1:], expected[1:])
        self.assertAlmostEqual(sequence.elapsed, elapsed)
        self.robot.send_jump()
        for at, angles, duration in sequence.commands:
            self.now = at + 1e-9
            self.robot.update_routine()
            self.assertEqual(self.robot.last_positions, angles)
            self.assertEqual(self.robot.last_duration, duration)
            if duration == 0:
                self.assertEqual(self.robot.sample_positions(), angles)
        self.now = 7.5
        self.robot.update_routine()
        self.assertFalse(self.robot.routine_active)

    def test_late_frames_keep_scheduled_time_without_accumulating_drift(self):
        self.robot.start_greeting(self.rest)
        self.now = 1.6
        self.robot.update_routine()
        # G2 began at 1.1, not at the late GUI update at 1.6.
        for actual, a, b in zip(self.robot.sample_positions(), G1, G2):
            self.assertAlmostEqual(actual, (a + b) / 2)
        self.now = 20
        self.robot.update_routine()
        self.assertFalse(self.robot.routine_active)
        self.assertEqual(self.robot.sample_positions(), self.rest)


class BodyContactTests(unittest.TestCase):
    def setUp(self):
        self.solver = k_solver()
        self.geometry = Q8FiveBarGeometry()
        self.anchors = KinematicPreview.ANCHORS

    def geometry_and_body(self, positions):
        legs = [self.geometry.solve(*positions[i:i+2]) for i in range(0, 8, 2)]
        return legs, support_transform(legs, self.anchors, KinematicPreview.MOUNT_HEIGHT)

    def world_foot(self, index, legs, body):
        x, side = self.anchors[index]
        px, py = legs[index].points[legs[index].foot]
        return body.apply(x + px, side, -py)

    def test_sitting_support_feet_stay_on_ground_with_rear_pair_anchored(self):
        target = list(self.solver.ik_solve(9.75, 60)[:2]) * 2 + list(self.solver.ik_solve(9.75, 20)[:2]) * 2
        legs, body = self.geometry_and_body(target)
        self.assertGreater(body.pitch, 0)
        self.assertNotAlmostEqual(body.height, 43.36)
        for i in range(4):
            self.assertAlmostEqual(self.world_foot(i, legs, body)[2], 0, places=6)
        for i in (2, 3):
            self.assertAlmostEqual(self.world_foot(i, legs, body)[0], -53)

    def test_rearing_rotates_and_translates_about_rear_supports(self):
        legs, body = self.geometry_and_body(RearingPose.target_positions())
        self.assertGreater(body.pitch, 0)
        self.assertNotAlmostEqual(body.x, 0)
        for i in (2, 3):
            self.assertAlmostEqual(self.world_foot(i, legs, body)[0], -53)
            self.assertAlmostEqual(self.world_foot(i, legs, body)[2], 0)
        for i in (0, 1):
            self.assertGreater(self.world_foot(i, legs, body)[2], 0)

    def test_pose_placement_is_continuous_and_symmetric_stances_restore_walking_view(self):
        target = list(self.solver.ik_solve(9.75, 60)[:2]) * 2 + list(self.solver.ik_solve(9.75, 20)[:2]) * 2
        for height in (25, 43.36, 60):
            rest = list(self.solver.ik_solve(9.75, height)[:2]) * 4
            _, neutral = self.geometry_and_body(rest)
            self.assertEqual((neutral.pitch, neutral.x, neutral.height), (0, 0, 43.36))
            previous = neutral
            for step in range(1, 101):
                sample = [a + (b - a) * step / 100 for a, b in zip(rest, target)]
                legs, body = self.geometry_and_body(sample)
                self.assertLess(abs(body.pitch - previous.pitch), .04)
                self.assertLess(abs(body.height - previous.height), 4)
                self.assertTrue(all(math.isfinite(v) for v in (body.pitch, body.x, body.height)))
                previous = body


if __name__ == '__main__':
    unittest.main()
