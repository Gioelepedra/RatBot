"""Command-to-linkage and timed-pose rendering tests, with no robot hardware."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'ratbot'))

import pygame
from demo_controller import DemoController
from gait_manager import GAITS, GaitManager
from kinematic_preview import KinematicPreview
from kinematics_solver import k_solver
from leg_geometry import LegGeometry, Q8FiveBarGeometry
from rearing_pose import RearingPose
from sitting_pose import SittingPose


class AnimationTests(unittest.TestCase):
    def setUp(self):
        pygame.init()
        self.addCleanup(pygame.quit)
        self.now = 0.0
        self.robot = DemoController(clock=lambda: self.now)
        self.geometry = Q8FiveBarGeometry()
        self.solver = k_solver()
        self.rest = list(self.solver.ik_solve(9.75, 43.36)[:2]) * 4

    def assert_geometry(self, angles):
        leg = self.geometry.solve(*angles)
        hips = leg.points[:2]
        knees = leg.points[2:4]
        foot = leg.points[leg.foot]
        self.assertEqual(hips, ((9.75, 0), (-9.75, 0)))
        for hip, knee, angle in zip(hips, knees, angles):
            self.assertAlmostEqual(math.dist(hip, knee), 25, places=6)
            self.assertAlmostEqual(math.dist(knee, foot), 40, delta=.002)
            self.assertAlmostEqual(knee[0] - hip[0], 25 * math.cos(math.radians(angle)))
            self.assertAlmostEqual(knee[1] - hip[1], 25 * math.sin(math.radians(angle)))
        # End effector satisfies the existing FK equations in the solver's frame.
        residual = self.solver._fk_calc((foot[0] + 9.75, foot[1]), *map(math.radians, angles))
        self.assertTrue(all(abs(value) < .17 for value in residual))
        return leg

    def test_every_gait_direction_keeps_commanded_angles_and_link_lengths(self):
        manager = GaitManager(self.solver)
        checked = set()
        preview = KinematicPreview(self.geometry)
        fixed_mounts = None
        for name in GAITS:
            self.assertTrue(manager.load_gait(name))
            for frames in manager.current_trajectories[name].values():
                for frame in frames:
                    for index in range(4):
                        pair = tuple(frame[index * 2:index * 2 + 2])
                        if pair not in checked:
                            leg = self.assert_geometry(pair)
                            expected_foot = self.solver.fk_solve(*pair)
                            self.assertAlmostEqual(leg.points[4][0] + 9.75, expected_foot[0], delta=.002)
                            self.assertAlmostEqual(leg.points[4][1], expected_foot[1], delta=.002)
                            checked.add(pair)
                    mounts = tuple(preview.project_leg(i, self.geometry.solve(*frame[i*2:i*2+2]))[:2]
                                   for i in range(4))
                    if fixed_mounts is None:
                        fixed_mounts = mounts
                    self.assertEqual(mounts, fixed_mounts)
        self.assertGreater(len(checked), 100)

    def test_timed_commands_interpolate_without_creating_commands(self):
        self.assertIsNone(self.robot.sample_positions())
        self.robot.move_all(self.rest, 1000, False)
        self.assertEqual(self.robot.sample_positions(), self.rest)  # No invented initial pose.
        target = list(self.solver.ik_solve(9.75, 60)[:2]) * 4
        self.robot.move_all(target, 1000, False)
        self.now = .5
        halfway = [(a + b) / 2 for a, b in zip(self.rest, target)]
        self.assertEqual(self.robot.sample_positions(), halfway)
        self.assertEqual(self.robot.last_positions, target)
        # A command that interrupts a transition begins at the current sample.
        self.robot.move_all(self.rest, 2000, False)
        self.assertEqual(self.robot.sample_positions(), halfway)
        self.now = 1.5
        self.assertEqual(self.robot.sample_positions(), [(a + b) / 2 for a, b in zip(halfway, self.rest)])
        self.now = 3
        self.assertEqual(self.robot.sample_positions(), self.rest)
        self.now = 100
        self.assertEqual(self.robot.sample_positions(), self.rest)
        self.assertEqual(self.robot.command_frames, 3)
        self.robot.move_all(target, 0, False)
        self.assertEqual(self.robot.sample_positions(), target)  # Gait frames are never smoothed.
        sample = self.robot.sample_positions()
        sample[0] = -999
        self.assertEqual(self.robot.sample_positions(), target)

    def check_transition(self, start):
        target = self.robot.last_positions[:]
        started = self.now
        duration = self.robot.last_duration / 1000
        count = self.robot.command_frames
        for fraction in (0, .25, .5, .75, 1):
            self.now = started + duration * fraction
            sample = self.robot.sample_positions()
            for actual, a, b in zip(sample, start, target):
                self.assertAlmostEqual(actual, a + (b - a) * fraction)
            for index in range(0, 8, 2):
                self.assert_geometry(sample[index:index+2])
        self.assertEqual(self.robot.command_frames, count)

    def test_sitting_and_rearing_use_actual_commands_through_each_transition(self):
        self.robot.move_all(self.rest, 0, False)
        sitting = SittingPose(self.solver, self.robot, lambda: self.now)
        for _ in range(2):
            start = self.robot.sample_positions()
            self.assertTrue(sitting.toggle(9.75, 43.36))
            self.check_transition(start)
        rearing = RearingPose(self.solver, self.robot, lambda: self.now)
        for _ in range(2):
            start = self.robot.sample_positions()
            self.assertTrue(rearing.toggle(9.75, 43.36))
            self.check_transition(start)
            while rearing._steps:
                self.now = rearing.transition_until
                start = self.robot.sample_positions()
                self.assertIsNone(rearing.update())
                self.check_transition(start)
        self.now += 10
        self.assertEqual(self.robot.sample_positions(), self.rest)
        self.assertFalse(rearing.blocks_movement())

    def test_renderer_uses_four_motor_pairs_and_draws_their_projected_feet(self):
        manager = GaitManager(self.solver)
        manager.load_gait('WALK')
        commands = manager.current_trajectories['WALK']['f'][7]
        preview = KinematicPreview(self.geometry)
        with patch.object(self.geometry, 'solve', wraps=self.geometry.solve) as solve, \
             patch.object(preview, '_circle', wraps=preview._circle) as circle:
            image = preview.render(commands, 'WALK')
            self.assertEqual([call.args for call in solve.call_args_list],
                             [tuple(commands[i:i+2]) for i in range(0, 8, 2)])
            for index in range(4):
                leg = self.geometry.solve(*commands[index*2:index*2+2])
                foot = preview.project_leg(index, leg)[leg.foot]
                self.assertIn(foot, [call.args[1] for call in circle.call_args_list])
        pixels = pygame.image.tobytes(image, 'RGB')
        self.assertEqual(pixels, pygame.image.tobytes(preview.render(commands, 'WALK'), 'RGB'))
        other = manager.current_trajectories['WALK']['f'][17]
        self.assertNotEqual(pixels, pygame.image.tobytes(preview.render(other, 'WALK'), 'RGB'))

    def test_renderer_accepts_another_link_graph_without_gui_changes(self):
        adapter = Mock()
        adapter.solve.return_value = LegGeometry(((0, 0), (10, 10), (-10, 10), (0, 30)),
                                                ((0, 1), (0, 2), (1, 3), (2, 3)), (0,), 3)
        preview = KinematicPreview(adapter)
        self.assertEqual(preview.render(self.rest, 'TROT').get_size(), (800, 480))
        self.assertEqual(adapter.solve.call_count, 4)


if __name__ == '__main__':
    unittest.main()
