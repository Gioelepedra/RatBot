'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script 
in your laptop with an ESP32C3 connected and control the robot via keyboard.
'''

import time
import pygame
import sys
import argparse
import os
from kinematics_solver import k_solver
from espnow import q8_espnow
from demo_controller import DemoController
from helpers import XiaoPortFinder, Q8Logger
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet
from input_handler import InputHandler, detect_and_init_joystick
from sitting_pose import SittingPose
from rearing_pose import RearingPose
from control_panel import ControlPanel, WINDOW_SIZE
import eth_theme as eth

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

# Pygame config
SPEED = 200
res = 0.2

# Helper Functions
def move_xy(x, y, dur = 0, deg = True):
    """Move robot legs to specific x,y position."""
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

# Parse command-line arguments
parser = argparse.ArgumentParser(description='RatBot control script')
parser.add_argument('com_port', nargs='?', help='COM port for ESP32C3 (optional, auto-detect if not provided)')
parser.add_argument('--debug', action='store_true', help='Enable debug logging')
parser.add_argument('--demo', action='store_true', help='Run the GUI without connecting to hardware (ignores COM port)')
args = parser.parse_args()

# Initialize logger
log = Q8Logger(debug=args.debug)

# Flags for main loop
movement = False
exit = False
record = False
request = "none"

# Find a serial port and connect
if args.demo:
    # Demo takes precedence even when a port was explicitly supplied.
    com_port = None
elif args.com_port:
    # User provided a COM port
    com_port = args.com_port
    if not XiaoPortFinder.validate(com_port):
        log.error(f"{com_port} is not an ESP32C3 or does not exist.")
        sys.exit(1)
else:
    # Auto-detect COM port
    com_port = XiaoPortFinder.find()
    if com_port is None:
        log.error("No ESP32C3 controller device found.")
        sys.exit(1)

# Start pygame instance
pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('RatBot - Gaits and poses')
clock = pygame.time.Clock()

# Set up pygame surface for logger
Q8Logger.set_pygame_surface(window)

# Detect and initialize input device (joystick or keyboard)
use_joystick, joystick, joystick_mapping = detect_and_init_joystick()
input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

# Resolve images for both source runs and PyInstaller executables.
def get_resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        # Running in development mode
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

control_panel = ControlPanel(get_resource_path(os.path.join('docs', 'poses')),
                             use_joystick, joystick_mapping, demo=args.demo)
log.info('Controls ready. G: switch gait. P: sit / stand. U: rear / lower.')

# Initialize kinamatics solver and Q8bot ESPNow instance
leg = k_solver(CENTER_DIST, L1, L2, L1, L2)
q8 = DemoController() if args.demo else q8_espnow(com_port)
sitting_pose = SittingPose(leg, q8)
rearing_pose = RearingPose(leg, q8)
q8.enable_torque()

# Initialize GaitManager
gait_names = list(GAITS.keys())
gait_manager = GaitManager(leg, GAITS)

# Starting location of leg end effector in x and y
first_gait_params = GAITS[gait_names[0]]
pos_x = first_gait_params[1]
pos_y = first_gait_params[2]
move_xy(pos_x, pos_y, 1000)

# Pre-calculate trajectories for default gait
if not gait_manager.load_gait(gait_names[0]):
    log.error(f"Failed to load default gait: {gait_names[0]}")
    sys.exit(1)

time.sleep(2)

demo_action_held = set()
while True:
    clock.tick(SPEED)
    events = pygame.event.get()
    if (any(event.type == pygame.QUIT or
            (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE)
            for event in events) or input_handler.is_action_pressed('exit')):
        break

    demo_pressed = set()
    if args.demo:
        q8.update_routine()
        held = {name for name in ('greet', 'jump') if input_handler.is_action_pressed(name)}
        demo_pressed = held - demo_action_held
        demo_action_held = held

    transition_error = rearing_pose.update()
    if transition_error:
        log.error(transition_error)

    # Consume both keys every frame so holding either never repeats a toggle.
    sit_pressed = input_handler.keyboard_action_pressed_once('sit', events)
    rear_pressed = input_handler.keyboard_action_pressed_once('rear', events)
    if args.demo and q8.routine_active:
        sit_pressed = rear_pressed = False
    if rear_pressed:
        if sitting_pose.blocks_movement():
            log.info('Press P to stand before rearing.')
        else:
            try:
                toggle = rearing_pose.retry_lowering if rearing_pose.failed else rearing_pose.toggle
                if toggle(pos_x, pos_y):
                    if record:
                        q8.finish_recording()
                    record = False
                    gait_manager.stop()
                    movement = False
                    log.info('Rearing pose' if rearing_pose.active else 'Lowering to gait position')
                else:
                    log.error('Could not send rearing/lowering pose')
            except ValueError as error:
                log.error(str(error))
    elif sit_pressed and rearing_pose.blocks_movement():
        log.info('Press U to lower before sitting.')
    elif sit_pressed:
        try:
            if sitting_pose.toggle(pos_x, pos_y):
                if record:
                    q8.finish_recording()
                record = False
                gait_manager.stop()
                movement = False
                log.info("Sitting pose" if sitting_pose.active else "Returning to gait position")
            else:
                log.error("Could not send sitting/standing pose")
        except ValueError as error:
            log.error(str(error))

    # Clear screen and render logger messages
    window.fill(eth.BACKGROUND)
    display_pose = rearing_pose if rearing_pose.blocks_movement() else sitting_pose
    control_panel.draw(window, gait_manager.current_gait, display_pose)
    Q8Logger.render_pygame_messages()  # Draw logger on top
    if not args.demo:
        pygame.display.flip()

    if args.demo and q8.routine_active:
        # Continue rendering and accepting exit while a scripted demo plays.
        pass
    elif sitting_pose.blocks_movement() or rearing_pose.blocks_movement():
        # Keep the static pose; battery checks and exit remain available.
        if input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
    elif movement:
        # Get requested direction from input handler
        requested_direction = input_handler.get_movement_direction()

        if requested_direction:
            # Start or switch movement direction
            if gait_manager.start_movement(requested_direction):
                # Execute current trajectory
                pos = gait_manager.tick()
                if pos:
                    q8.move_all(pos, 0, record)
            else:
                # Failed to start movement
                movement = False
        else:
            # No movement input - transition to idle
            move_xy(pos_x, pos_y, 0)
            q8.finish_recording()
            record = False
            gait_manager.stop()
            movement = False
    else:
        # Check for movement input
        if input_handler.is_movement_input():
            movement = True
        # Check action inputs using generalized interface
        elif input_handler.is_action_pressed('reset'):
            log.info("Gait Reset")
            move_xy(pos_x, pos_y, 500)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('recover'):
            log.info("Recovering servos: clearing faults and re-enabling torque")
            q8.send_recover()
            time.sleep(2)
            move_xy(pos_x, pos_y, 1000)
        elif ('jump' in demo_pressed if args.demo else input_handler.is_action_pressed('jump')):
            log.info("Jump")
            if args.demo:
                q8.start_jump(list(leg.ik_solve(pos_x, pos_y, True, 1)[:2]) * 4)
            else:
                q8.send_jump()
                time.sleep(5)
                move_xy(pos_x, pos_y, 500)
        elif input_handler.is_action_pressed('switch_gait'):
            # Cycle to next gait
            gait_names.append(gait_names.pop(0))
            new_gait = gait_names[0]

            # Load new gait
            if gait_manager.load_gait(new_gait):
                # Update position to match new gait
                pos_x, pos_y = GAITS[new_gait][1], GAITS[new_gait][2]
                move_xy(pos_x, pos_y, 500)
                log.info(f"Switched to {new_gait}")
            else:
                log.error(f"Failed to load gait: {new_gait}")
                gait_names.insert(0, gait_names.pop())  # Revert gait change
            time.sleep(0.2)
        elif input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('record'):
            log.debug("Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('show_range'):
            log.info("Show Range")
            show_range(q8)
            time.sleep(0.2)
        elif ('greet' in demo_pressed if args.demo else input_handler.is_action_pressed('greet')):
            log.info("Greet")
            if args.demo:
                q8.start_greeting(list(leg.ik_solve(pos_x, pos_y, True, 1)[:2]) * 4)
            else:
                greet(q8)
                move_xy(pos_x, pos_y, 1000)  # Return to rest position
                time.sleep(0.2)
        elif input_handler.is_action_pressed('exit'):
            break

    # Drain replies while sitting as well as while operating a gait.
    try:
        while q8.serialHandler.in_waiting > 0:
            raw_data = q8.serialHandler.readline().decode('utf-8').strip().split()
            if request == "battery" and raw_data and raw_data[0].isdigit():
                log.info(f"Battery: {int(raw_data[0])}%")
                request = "none"
    except (ValueError, UnicodeError):
        log.debug("Data reading failed. Continuing...")

    if args.demo:
        # Render after processing input so release/pose blocking shows IDLE immediately.
        direction = gait_manager.get_current_direction() if movement and gait_manager.is_moving() else None
        control_panel.draw_demo_robot(window, q8.sample_positions(), gait_manager.current_gait, display_pose,
                                      q8.routine)
        control_panel.draw_demo_status(window, gait_manager.current_gait, direction, q8.command_frames)
        pygame.display.flip()

q8.disable_torque()
if joystick:
    joystick.quit()
pygame.quit()
