"""Run the real GUI and keyboard handlers with all serial access forbidden."""
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

from collections import defaultdict
from contextlib import ExitStack
from pathlib import Path
import runpy
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'ratbot'))

import pygame
import eth_theme as eth
from control_panel import ControlPanel
from demo_controller import DemoController
from gait_manager import GAITS, GaitManager
from input_handler import InputHandler
from kinematic_preview import KinematicPreview
from rearing_pose import RearingPose
from sitting_pose import SittingPose


class DemoModeTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.addCleanup(pygame.quit)
        # Raise immediately AND assert no calls, even if application code catches errors.
        # Patch methods before the constructor, using PySerial's platform-neutral API.
        for target in ('serial.Serial.open', 'serial.Serial.write',
                       'serial.Serial', 'serial.serial_for_url',
                       'serial.tools.list_ports.comports'):
            guard = self.stack.enter_context(patch(target, side_effect=AssertionError(target)))
            self.addCleanup(guard.assert_not_called)
        self.stack.enter_context(patch('time.sleep'))
        self.stack.enter_context(patch('pygame.joystick.get_count', return_value=0))

    def run_gui(self, arguments):
        with patch.object(sys, 'argv', ['operate.py', *arguments]):
            return runpy.run_path(str(ROOT / 'ratbot' / 'operate.py'), run_name='__main__')

    def test_keyboard_movement_is_detected_while_held_and_clears_on_release(self):
        handler = InputHandler()
        for key, direction in ((pygame.K_w, 'f'), (pygame.K_a, 'l'),
                               (pygame.K_s, 'b'), (pygame.K_d, 'r'),
                               (pygame.K_q, 'fl_0.75'), (pygame.K_e, 'fr_0.75')):
            with self.subTest(key=key):
                with patch.object(pygame.key, 'get_pressed', return_value=defaultdict(bool, {key: True})):
                    for _ in range(5):
                        self.assertTrue(handler.is_movement_input())
                        self.assertEqual(handler.get_movement_direction(), direction)
                with patch.object(pygame.key, 'get_pressed', return_value=defaultdict(bool)):
                    self.assertFalse(handler.is_movement_input())
                    self.assertIsNone(handler.get_movement_direction())

    def test_held_movement_sends_gait_frames_and_updates_demo_display(self):
        for switch_gait in (False, True):
            with self.subTest(switch_gait=switch_gait), ExitStack() as stack:
                robot = DemoController()
                frame = [0]
                snapshots = {}
                keys = {2: pygame.K_g} if switch_gait else {}
                movements = [(pygame.K_w, 'f', 'FORWARD'), (pygame.K_a, 'l', 'LEFT'),
                             (pygame.K_s, 'b', 'BACKWARD'), (pygame.K_d, 'r', 'RIGHT'),
                             (pygame.K_q, 'fl_0.75', 'FORWARD LEFT'),
                             (pygame.K_e, 'fr_0.75', 'FORWARD RIGHT')]
                for index, (key, _, _) in enumerate(movements):
                    start = 4 + index * 8
                    keys.update({n: key for n in range(start, start + 6)})

                def events():
                    frame[0] += 1
                    self.assertLessEqual(frame[0], 52)
                    return [pygame.event.Event(pygame.QUIT)] if frame[0] == 52 else []

                def pressed():
                    return defaultdict(bool, {keys.get(frame[0]): True})

                draw_status = ControlPanel.draw_demo_status

                def status(panel, window, gait, direction, count):
                    draw_status(panel, window, gait, direction, count)
                    snapshots[frame[0]] = (gait, direction, count, robot.last_positions[:],
                                          rendered.call_args.args[1][:])
                    # Confirm the counter is rendered from commands, not elapsed GUI frames.
                    label = panel.fonts[23].render(f'Joint-command frames: {robot.command_frames}',
                                                  True, eth.TEXT_SECONDARY)
                    expected = pygame.Surface(label.get_size())
                    expected.fill(eth.BACKGROUND)
                    expected.blit(label, (0, 0))
                    rect = label.get_rect(topleft=(1240 - label.get_width(), 652))
                    self.assertEqual(pygame.image.tobytes(window.subsurface(rect), 'RGB'),
                                     pygame.image.tobytes(expected, 'RGB'))

                stack.enter_context(patch('demo_controller.DemoController', return_value=robot))
                stack.enter_context(patch.object(pygame.event, 'get', side_effect=events))
                stack.enter_context(patch.object(pygame.key, 'get_pressed', side_effect=pressed))
                stack.enter_context(patch.object(ControlPanel, 'draw_demo_status', status))
                text = stack.enter_context(patch.object(ControlPanel, '_text', autospec=True,
                                                        side_effect=ControlPanel._text))
                ticks = stack.enter_context(patch.object(GaitManager, 'tick', autospec=True,
                                                         side_effect=GaitManager.tick))
                rendered = stack.enter_context(patch.object(KinematicPreview, 'render', autospec=True,
                                                            side_effect=KinematicPreview.render))
                result = self.run_gui(['--demo'])
                gait = 'TROT_HIGH' if switch_gait else 'TROT'
                trajectories = result['gait_manager'].current_trajectories[gait]
                self.assertEqual(snapshots[1][1:3], (None, 1))  # Startup rest command only.
                for index, (_, direction, label) in enumerate(movements):
                    start = 4 + index * 8
                    baseline = snapshots[start][2]
                    for offset in range(5):
                        sample = snapshots[start + 1 + offset]
                        self.assertEqual(sample[:3], (gait, direction, baseline + offset + 1))
                        self.assertEqual(sample[3], trajectories[direction][offset])
                        self.assertEqual(sample[4], trajectories[direction][offset])
                    # Release sends one return-to-rest command, then the counter stays still.
                    self.assertEqual(snapshots[start + 6][1:3], (None, baseline + 6))
                    self.assertEqual(snapshots[start + 7][1:3], (None, baseline + 6))
                    self.assertEqual(snapshots[start + 6][3], snapshots[start + 6][4])
                    self.assertEqual(snapshots[start + 6][4], snapshots[start + 7][4])
                    self.assertIn(f'{label} — {gait}', [call.args[2] for call in text.call_args_list])
                self.assertIn(f'IDLE — {gait}', [call.args[2] for call in text.call_args_list])
                self.assertEqual(ticks.call_count, 30)
                self.assertEqual(robot.command_frames, 37 + int(switch_gait))

    def test_gui_renders_intermediate_sitting_and_rearing_commands(self):
        frame = [0]
        clock = lambda: frame[0] * .25
        robot = DemoController(clock=clock)
        rendered = {}
        downs = {2: pygame.K_p, 8: pygame.K_p, 14: pygame.K_u, 34: pygame.K_u}

        def events():
            frame[0] += 1
            self.assertLessEqual(frame[0], 70)
            if frame[0] == 70:
                return [pygame.event.Event(pygame.QUIT)]
            if frame[0] in downs:
                return [pygame.event.Event(pygame.KEYDOWN, key=downs[frame[0]])]
            if frame[0] - 1 in downs:
                return [pygame.event.Event(pygame.KEYUP, key=downs[frame[0] - 1])]
            return []

        original_render = KinematicPreview.render

        def render(preview, positions, name):
            self.assertEqual(positions, robot.sample_positions())
            rendered[frame[0]] = positions[:]
            return original_render(preview, positions, name)

        with patch('demo_controller.DemoController', return_value=robot), \
             patch('sitting_pose.SittingPose', side_effect=lambda leg, bot: SittingPose(leg, bot, clock)), \
             patch('rearing_pose.RearingPose', side_effect=lambda leg, bot: RearingPose(leg, bot, clock)), \
             patch.object(pygame.event, 'get', side_effect=events), \
             patch.object(pygame.key, 'get_pressed', return_value=defaultdict(bool)), \
             patch.object(KinematicPreview, 'render', render):
            self.run_gui(['--demo'])
        self.assertNotEqual(rendered[2], rendered[3])  # Sitting transition, not a static image switch.
        self.assertNotEqual(rendered[3], rendered[6])
        self.assertNotEqual(rendered[14], rendered[15])  # Rearing entry.
        self.assertNotEqual(rendered[34], rendered[35])  # Lowering.
        for n in range(60, 70):
            self.assertEqual(rendered[n], robot.last_positions)
        self.assertEqual(robot.command_frames, 9)  # Startup, sit/stand, three entry + three exit.

    def test_demo_keyboard_gaits_poses_and_actions_never_touch_serial(self):
        for arguments in (['--demo'], ['mock-port', '--demo', '--debug']):
            with self.subTest(arguments=arguments), ExitStack() as stack:
                for target in ('helpers.XiaoPortFinder.find', 'helpers.XiaoPortFinder.validate',
                               'espnow.q8_espnow'):
                    guard = stack.enter_context(patch(target, side_effect=AssertionError(target)))
                    stack.callback(guard.assert_not_called)
                frame = [0]
                seen = []
                statuses = {}
                robot = DemoController(clock=lambda: frame[0] * 10)
                stack.enter_context(patch('demo_controller.DemoController', return_value=robot))
                keys = {n: pygame.K_g for n in range(1, 9)}
                keys.update(dict(zip(range(9, 15), (pygame.K_w, pygame.K_s, pygame.K_a,
                                                    pygame.K_d, pygame.K_q, pygame.K_e))))
                keys.update({16: pygame.K_p, 17: pygame.K_b, 18: pygame.K_p,
                             20: pygame.K_u, 23: pygame.K_b, 24: pygame.K_u,
                             28: pygame.K_z, 29: pygame.K_w, 30: pygame.K_w,
                             32: pygame.K_r, 33: pygame.K_x, 34: pygame.K_j,
                             35: pygame.K_c, 36: pygame.K_h, 37: pygame.K_b,
                             38: pygame.K_ESCAPE})

                def events():
                    frame[0] += 1
                    n = frame[0]
                    self.assertLessEqual(n, 38, 'GUI failed to exit')
                    result = []
                    if n - 1 in keys:
                        result.append(pygame.event.Event(pygame.KEYUP, key=keys[n - 1]))
                    if n in keys:
                        result.append(pygame.event.Event(pygame.KEYDOWN, key=keys[n]))
                    return result

                def pressed():
                    held = defaultdict(bool, {keys.get(frame[0]): True})
                    if frame[0] in (16, 17, 18, 20, 21, 22, 23, 24, 25, 26):
                        held[pygame.K_w] = True  # Poses must keep the movement display idle.
                    return held

                original_draw = ControlPanel.draw

                def draw(panel, window, gait, pose):
                    original_draw(panel, window, gait, pose)
                    seen.append(panel.active_preview)
                    # Check the actual status-bar pixels on every frame.
                    label = panel.fonts[23].render('DEMO MODE - NO HARDWARE CONNECTED',
                                                  True, eth.TEXT_ON_ACCENT)
                    expected = pygame.Surface(label.get_size())
                    expected.fill(eth.STATUS_BAR)
                    expected.blit(label, (0, 0))
                    rect = label.get_rect(topleft=(window.get_width() - label.get_width() - 24, 770))
                    self.assertEqual(pygame.image.tobytes(window.subsurface(rect), 'RGB'),
                                     pygame.image.tobytes(expected, 'RGB'))

                original_status = ControlPanel.draw_demo_status

                def status(panel, window, gait, direction, count):
                    original_status(panel, window, gait, direction, count)
                    statuses[frame[0]] = direction

                stack.enter_context(patch.object(pygame.event, 'get', side_effect=events))
                stack.enter_context(patch.object(pygame.key, 'get_pressed', side_effect=pressed))
                stack.enter_context(patch.object(ControlPanel, 'draw', draw))
                stack.enter_context(patch.object(ControlPanel, 'draw_demo_status', status))
                # Advance pose time without changing any transition parameters.
                stack.enter_context(patch('sitting_pose.SittingPose', side_effect=lambda leg, robot:
                                          SittingPose(leg, robot, lambda: frame[0] * 10)))
                stack.enter_context(patch('rearing_pose.RearingPose', side_effect=lambda leg, robot:
                                          RearingPose(leg, robot, lambda: frame[0] * 10)))
                moves = stack.enter_context(patch.object(DemoController, 'move_all', autospec=True,
                                                         side_effect=DemoController.move_all))
                result = self.run_gui(arguments)
                self.assertIsInstance(result['q8'], DemoController)
                self.assertFalse(result['q8'].torque_on)
                self.assertEqual(set(seen), set(GAITS) | {'SITTING', 'REARING'})
                self.assertFalse(result['sitting_pose'].blocks_movement())
                self.assertFalse(result['rearing_pose'].blocks_movement())
                self.assertTrue(any(call.args[2:] == (0, True) for call in moves.call_args_list))
                self.assertGreater(moves.call_count, 30)  # Includes movement and scripted routines.
                self.assertEqual(result['q8'].command_frames, moves.call_count)
                self.assertTrue(all(statuses[n] is None for n in range(16, 28)))
                self.assertEqual(result['q8'].serialHandler.in_waiting, 0)
                self.assertEqual(result['q8'].serialHandler.readline(), b'')

    def test_demo_window_close_releases_mock_torque(self):
        with patch.object(pygame.event, 'get', return_value=[pygame.event.Event(pygame.QUIT)]), \
             patch('espnow.q8_espnow', side_effect=AssertionError('hardware controller')) as hardware:
            result = self.run_gui(['--demo'])
        hardware.assert_not_called()
        self.assertIsInstance(result['q8'], DemoController)
        self.assertFalse(result['q8'].torque_on)

    def test_greeting_and_jump_render_every_frame_and_remain_exitable(self):
        for key, name, frames, commands in ((pygame.K_h, 'GREETING', 146, 12),
                                            (pygame.K_j, 'JUMP', 164, 6)):
            for early_exit in (False, True):
                with self.subTest(routine=name, early_exit=early_exit):
                    frame = [0]
                    robot = DemoController(clock=lambda: frame[0] * .05)
                    samples = []
                    end = 10 if early_exit else frames

                    def events():
                        frame[0] += 1
                        self.assertLessEqual(frame[0], end)
                        return [pygame.event.Event(pygame.QUIT)] if frame[0] == end else []

                    def pressed():
                        # Hold the routine key through completion: it must not restart.
                        keys = defaultdict(bool, {key: True})
                        if 4 <= frame[0] <= 8:
                            keys[pygame.K_w] = True  # Walking must not interrupt the script.
                        return keys

                    original_render = KinematicPreview.render

                    def render(preview, positions, label):
                        samples.append(positions[:])
                        self.assertEqual(positions, robot.sample_positions())
                        return original_render(preview, positions, label)

                    with patch('demo_controller.DemoController', return_value=robot), \
                         patch.object(pygame.event, 'get', side_effect=events), \
                         patch.object(pygame.key, 'get_pressed', side_effect=pressed), \
                         patch.object(KinematicPreview, 'render', render), \
                         patch('time.sleep') as sleep:
                        self.run_gui(['--demo'])
                    self.assertEqual(len(samples), end - 1)
                    self.assertEqual(robot.routine.name, name)
                    self.assertFalse(robot.torque_on)
                    sleep.assert_called_once_with(2)  # Existing startup wait only.
                    if early_exit:
                        self.assertTrue(robot.routine_active)
                        self.assertEqual(robot.command_frames, 2)
                    else:
                        self.assertFalse(robot.routine_active)
                        self.assertEqual(robot.command_frames, commands)
                        self.assertGreater(len({tuple(p) for p in samples}), 15)

    def test_without_demo_keeps_port_selection_and_hardware_controller(self):
        for arguments in ([], ['mock-port']):
            with self.subTest(arguments=arguments), \
                 patch('helpers.XiaoPortFinder.find', return_value='detected-port') as find, \
                 patch('helpers.XiaoPortFinder.validate', return_value=True) as validate, \
                 patch('espnow.q8_espnow', return_value=DemoController()) as hardware, \
                 patch('demo_controller.DemoController', side_effect=AssertionError('demo selected')), \
                 patch('kinematic_preview.KinematicPreview', side_effect=AssertionError('demo renderer selected')), \
                 patch.object(pygame.event, 'get', return_value=[pygame.event.Event(pygame.QUIT)]):
                result = self.run_gui(arguments)
                self.assertFalse(result['control_panel'].demo)
                self.assertIsNone(result['control_panel'].animation)
                if arguments:
                    validate.assert_called_once_with('mock-port')
                    find.assert_not_called()
                    hardware.assert_called_once_with('mock-port')
                else:
                    find.assert_called_once_with()
                    validate.assert_not_called()
                    hardware.assert_called_once_with('detected-port')


if __name__ == '__main__':
    unittest.main()
