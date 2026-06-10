#!/usr/bin/env python3
"""
orchestrator.py - Craps Dealer action orchestrator (multi-phase)
Sequences defined inline; phase specified via ROS2 parameter.

Usage:
    ros2 launch soa_bringup bi_soa_bringup.launch.py
    ros2 run soa_apps orchestrator --ros-args -p phase:=dice_sequence
    ros2 run soa_apps orchestrator --ros-args -p phase:=chip_collect
"""
import csv
import os
import time
import shutil
import signal
import datetime
import threading
import subprocess

import rclpy
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import SwitchController, ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image

import numpy as np
import cv2
from ultralytics import YOLO

# ══════════════════════════════════════════════════════════════════
#  Global config
# ══════════════════════════════════════════════════════════════════
BASE          = '/home/ubuntu/techin517'
JS            = f'{BASE}/joint_states'   # joint_states CSV dir shorthand

# Dice detection
DICE_MODEL_PATH = f'{BASE}/dice-recognition/runs/detect/train/weights/best.pt'
DICE_ROI        = (285, 206, 381, 286)
DICE_SCALE      = 8
DICE_SETTLE_SEC = 2.0

# Bet detection
# Bet zones (overhead-camera pixel ROIs): SMALL on the yellow zone, BIG on the blue zone.
YELLOW_ROI        = (31, 193, 150, 376)   # SMALL bet zone
BLUE_ROI          = (170, 213, 242, 395)  # BIG bet zone
# Chip detection is color-based: count ORANGE pixels (the chip color) inside an ROI.
# HSV, OpenCV H in 0..179. The tray is YELLOW, and yellow sits just above orange in hue,
# so the upper hue bound stays below yellow (~22) to avoid false positives.
# Tuning: if the yellow tray triggers, lower ORANGE_HSV_HIGH[0]; if chips are missed, raise it.
ORANGE_HSV_LOW    = (5, 120, 80)
ORANGE_HSV_HIGH   = (18, 255, 255)
BET_THRESHOLD     = 0.03   # fraction of ROI pixels that must be orange to count as a chip
BET_DEBUG         = True   # log live orange fractions while waiting for a bet (for tuning)
# A bet is only locked after the orange fraction has stayed above threshold (in one zone)
# continuously for this long, so a chip still being placed / moved is not locked prematurely.
BET_STABLE_SEC    = 2.0
BET_TIMEOUT_SEC   = 30.0

# Camera
# bringup's cameras.launch.py names the overhead camera 'overhead_cam',
# namespace=static_camera, so the real topic is the one below (not static_camera/static_camera).
CAM_TOPIC = '/static_camera/overhead_cam/color/image_raw'

# Gripper
GRIPPER_ACTION = {
    'left':  '/follower/left_gripper_controller/gripper_cmd',
    'right': '/follower/right_gripper_controller/gripper_cmd',
}
GRIPPER_MAX_EFFORT = 1.0

# ══════════════════════════════════════════════════════════════════
#  Sequences  (defined inline here; no YAML file needed)
# ══════════════════════════════════════════════════════════════════
SEQUENCES = {

    'dice_sequence': [
        {
            'name':       'raise_for_dice',
            'left':       f'{JS}/raise_for_dice_left.csv',
            'right':      f'{JS}/raise_for_dice_right.csv',
            'secs':       1.0,
            'wait_after': 0.1,
        },
        {
            'name':       'pickup_dice_1',
            'left':       f'{JS}/pickup_dice_left.csv',
            'secs':       1.2,
            'wait_after': 0.5,
        },
        {
            'name':       'collect_dice_1',
            'left':       f'{JS}/collect_dice_left.csv',
            'right':      f'{JS}/collect_dice_right.csv',
            'secs':       0.5,
            'wait_after': 0.1,
        },
        {
            'name':       'pickup_dice_2',
            'left':       f'{JS}/pickup_dice_left.csv',
            'secs':       1.2,
            'wait_after': 0.5,
        },
        {
            'name':       'collect_dice_2',
            'left':       f'{JS}/collect_dice_left.csv',
            'right':      f'{JS}/collect_dice_right.csv',
            'secs':       0.5,
            'wait_after': 0.1,
        },
        {
            'name':       'drop_dice',
            'left':       f'{JS}/drop_dice_left.csv',
            'right':      f'{JS}/drop_dice_right.csv',
            'secs':       1.0,
            'wait_after': 0.1,
            'read_dice':  True,
        },
    ],

    'chip_dispense': [
        {
            'name':       'chip_dispense',
            'left':       f'{JS}/collect_chip_left.csv',
            'right':      f'{JS}/collect_chip_right.csv',
            'secs':       1.0,
            'wait_after': 0.1,
            'read_dice':  True,
        },
    ],
}

# ══════════════════════════════════════════════════════════════════
#  Native lerobot ACT policy phases (NO rosetta)
#  Hard handoff to native lerobot inference. Flow: kill bringup to free devices, then run lerobot ACT.
# ══════════════════════════════════════════════════════════════════
#
#  ACT is single-arm (so101_follower, left arm). Confirmed real wiring:
#     follower_left = /dev/ttyACM0   (ACT robot)
#     follower_right= /dev/ttyACM1
#     leader_left   = /dev/ttyACM2   (ACT teleop)
#     NOTE: bi_soa_params.yaml must match the above (left follower=ACM0,
#           right follower=ACM1, left leader=ACM2), or bringup opens the wrong ports.
#
# ROS nodes holding USB devices; kill before lerobot takes over (unmanaged-mode fallback)
ROS_DEVICE_HOLDERS = [
    'ros2_control_node',     # holds follower serial ports (feetech)
    'realsense2_camera',     # holds overhead RealSense
    'usb_cam',               # holds wrist cams
]

# follower serial ports held by bringup (confirm freed after stopping bringup)
FOLLOWER_PORTS = ['/dev/ttyACM0', '/dev/ttyACM1']   # [left_follower, right_follower]

# Bringup launch, started/stopped by the orchestrator itself (when manage_bringup:=true).
# controller:=jtc -> arm(JTC)+gripper(action) controllers are active from the start,
# so no controller hot-switch is needed, avoiding the switch-before-spawn race.
BRINGUP_CMD = [
    'ros2', 'launch', 'soa_bringup', 'bi_soa_bringup.launch.py',
    'controller:=jtc',
]
JOINT_STATES_TOPIC = '/follower/joint_states'   # readiness probe

# controllers that must be active for the game (CSV/round)
JTC_CONTROLLERS = [
    'left_arm_controller', 'right_arm_controller',
    'left_gripper_controller', 'right_gripper_controller',
]

# -- Single-arm ACT inference params (mirror the verified lerobot-record command) --
ACT_ROBOT_PORT      = '/dev/ttyACM0'          # follower
ACT_TELEOP_PORT     = '/dev/ttyACM2'          # leader (teleop)
ACT_ROBOT_ID        = 'follower_left'
ACT_TELEOP_ID       = 'leader_left'
ACT_ROBOT_CALIB     = f'{BASE}/huggingface/lerobot/calibration/robots/so101_follower'
ACT_TELEOP_CALIB    = f'{BASE}/huggingface/lerobot/calibration/teleoperators/so101_leader'
# serial ports that must be free before ACT takes over
ACT_REQUIRED_PORTS  = [ACT_ROBOT_PORT, ACT_TELEOP_PORT]
# camera keys/count/order/resolution/fps must exactly match the training dataset (note overhead fps=15)
ACT_CAMERAS = (
    "{wrist: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, fourcc: MJPG}, "
    "overhead: {type: intelrealsense, serial_number_or_name: '241122070943', width: 640, height: 480, fps: 15}}"
)

LEROBOT_POLICIES = {
    'chip_collect': {
        'policy_path':    f'{BASE}/craps-chip-collect-v3-act-train-v2/checkpoints/last/pretrained_model',
        'task':           'pick up the chip from the tray and place it into the rail entrance',
        'repo_id':        'gix/eval_craps_chip_collect_act',
        'dataset_root':   f'{BASE}/eval-craps-chip-collect-act',
        'episodes':       1,        # run once per round in the game
        'episode_time_s': 18,
    },
}

# ══════════════════════════════════════════════════════════════════
JOINT_COLS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
ARM_JOINTS = {
    'left':  ['left_shoulder_pan',  'left_shoulder_lift',  'left_elbow_flex',  'left_wrist_flex',  'left_wrist_roll'],
    'right': ['right_shoulder_pan', 'right_shoulder_lift', 'right_elbow_flex', 'right_wrist_flex', 'right_wrist_roll'],
}
ARM_TOPIC = {
    'left':  '/follower/left_arm_controller/joint_trajectory',
    'right': '/follower/right_arm_controller/joint_trajectory',
}


def load_rows(path: str) -> list:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def make_trajectory(joint_names, positions, secs) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = joint_names
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    msg.points = [pt]
    return msg


class Orchestrator(Node):

    def __init__(self):
        super().__init__('orchestrator')

        self.declare_parameter('phase', '')
        self.declare_parameter('manage_bringup', True)
        self.declare_parameter('bringup_args', '')
        phase = self.get_parameter('phase').get_parameter_value().string_value
        if not phase:
            self.get_logger().error(
                f'No phase specified! Use: --ros-args -p phase:=<name>\n'
                f'Available: {["round"] + list(SEQUENCES.keys()) + list(LEROBOT_POLICIES.keys())}'
            )
            raise ValueError('phase parameter required')
        if phase != 'round' and phase not in SEQUENCES and phase not in LEROBOT_POLICIES:
            self.get_logger().error(
                f'Unknown phase: "{phase}". '
                f'Available: {["round"] + list(SEQUENCES.keys()) + list(LEROBOT_POLICIES.keys())}'
            )
            raise ValueError(f'Unknown phase: {phase}')

        self._phase = phase
        self._is_round = phase == 'round'
        self._is_policy_phase = phase in LEROBOT_POLICIES

        self._manage_bringup = self.get_parameter('manage_bringup').get_parameter_value().bool_value
        self._bringup_args = self.get_parameter('bringup_args').get_parameter_value().string_value
        self._bringup_proc = None

        if self._is_round:
            self._sequence = None
            self.get_logger().info('Phase "round": full game (bet → dice → resolve → collect/dispense)')
        elif self._is_policy_phase:
            self._policy_cfg = LEROBOT_POLICIES[phase]
            self._sequence = None
            self.get_logger().info(
                f'Phase "{phase}" is a native lerobot ACT policy phase (no rosetta)'
            )
        else:
            self.get_logger().info(f'Loading CSV sequence: {phase}')
            self._sequence = SEQUENCES[phase]
            self.get_logger().info(f'Loaded {len(self._sequence)} actions')

        self._arm_pubs = {
            arm: self.create_publisher(JointTrajectory, topic, 10)
            for arm, topic in ARM_TOPIC.items()
        }
        self._gripper_clients = {
            arm: ActionClient(self, GripperCommand, action)
            for arm, action in GRIPPER_ACTION.items()
        }
        self._switch_client = self.create_client(
            SwitchController,
            '/follower/controller_manager/switch_controller'
        )
        self._list_client = self.create_client(
            ListControllers,
            '/follower/controller_manager/list_controllers'
        )

        self._latest_frame = None
        self._cam_sub = self.create_subscription(
            Image, CAM_TOPIC, self._cam_callback, 1)

        self._player_bet = None
        self._abort = False        # set True to stop the whole game loop (e.g. chip removed)

    # ── Camera ──────────────────────────────────────────────────
    def _cam_callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self._latest_frame = img[:, :, ::-1].copy()

    def _get_fresh_frame(self, timeout=5.0):
        self._latest_frame = None
        deadline = time.time() + timeout
        while self._latest_frame is None and time.time() < deadline:
            time.sleep(0.05)
        return self._latest_frame

    # ── Bet detection ────────────────────────────────────────────
    def _check_chip(self, img, roi):
        """Return the fraction of ORANGE pixels (chip color) inside the ROI.
        HSV-based; the hue range stays below the yellow tray to avoid false positives."""
        x1, y1, x2, y2 = roi
        region = img[y1:y2, x1:x2]
        if region.size == 0:
            return 0.0
        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(ORANGE_HSV_LOW,  dtype=np.uint8),
                           np.array(ORANGE_HSV_HIGH, dtype=np.uint8))
        return float(np.count_nonzero(mask)) / mask.size

    def _read_bet(self):
        """Block until a bet is placed. Rules:
        - the orange fraction in ONE zone must stay above threshold continuously for
          >= BET_STABLE_SEC before locking (so a chip still being placed is not locked early).
        - chips at both positions (double bet) -> do not proceed; remind to bet on one; keep waiting.
        - nothing detected, or the detection drops / switches zone -> reset the timer.
        Once locked, even if the player moves the chip to cheat, settlement uses the locked value."""
        self.get_logger().info('Waiting for player to place a bet (blocking until detected)...')
        self._player_bet = None
        warned_double = False
        last_dbg = 0.0
        candidate = None        # 'SMALL' / 'BIG' currently detected
        candidate_since = None  # time the current candidate first appeared
        while rclpy.ok() and self._player_bet is None:
            frame = self._get_fresh_frame(timeout=2.0)
            if frame is None:
                continue
            s_frac = self._check_chip(frame, YELLOW_ROI)
            b_frac = self._check_chip(frame, BLUE_ROI)
            small = s_frac > BET_THRESHOLD
            big   = b_frac > BET_THRESHOLD

            if small and big:
                # Double bet: warn immediately, never lock; reset timer.
                candidate = None
                candidate_since = None
                if not warned_double:
                    self.get_logger().warn('Chips on BOTH SMALL and BIG; bet on only ONE.')
                    print('\n' + '!'*44)
                    print('  Double bet detected: please bet on only ONE position (SMALL or BIG)')
                    print('!'*44 + '\n')
                    warned_double = True
                continue
            warned_double = False   # reset so a later double bet warns again

            current = 'SMALL' if small else 'BIG' if big else None
            if current != candidate:
                # Detection started, dropped, or switched zone -> restart the timer.
                candidate = current
                candidate_since = time.time() if current is not None else None
            held_for = (time.time() - candidate_since) if candidate_since is not None else 0.0

            if BET_DEBUG and time.time() - last_dbg > 1.0:
                self.get_logger().info(
                    f'orange frac  SMALL={s_frac:.3f}  BIG={b_frac:.3f}  (thr={BET_THRESHOLD})  '
                    f'{candidate or "-"} held={held_for:.1f}/{BET_STABLE_SEC}s')
                last_dbg = time.time()

            # Lock only after the same single-zone detection has held long enough.
            if candidate is not None and held_for >= BET_STABLE_SEC:
                self._player_bet = candidate

        if self._player_bet:
            print('\n' + '='*40)
            print(f'  Player bet (locked): {self._player_bet}')
            print('='*40 + '\n')

    def _verify_chip_present(self, checks=4, timeout=3.0):
        """Settlement check: is there still a chip on the tray (SMALL or BIG)?
        Multi-frame sampling: a chip seen in any frame counts as present; only if no frame shows a chip is it judged removed.
        If there is no camera frame at all, do not false-alarm (assume present)."""
        saw_frame = False
        for _ in range(checks):
            frame = self._get_fresh_frame(timeout=timeout)
            if frame is None:
                continue
            saw_frame = True
            small = self._check_chip(frame, YELLOW_ROI) > BET_THRESHOLD
            big   = self._check_chip(frame, BLUE_ROI)   > BET_THRESHOLD
            if small or big:
                return True          # chip still on tray (may have been moved to cheat, but locked value is used)
            time.sleep(0.2)
        if not saw_frame:
            self.get_logger().warn('No camera frame for chip verification; assuming present')
            return True
        return False                  # no chip across all frames -> removed early

    def _alarm_chip_removed(self):
        self.get_logger().error('CHIP REMOVED before settlement!')
        print('\n' + '#'*50)
        print('  ALARM: no chip on the tray at settlement; it may have been removed early!')
        print('  Stopping the game process.')
        print('#'*50 + '\n')

    # ── Dice detection ───────────────────────────────────────────
    def _read_dice(self):
        self.get_logger().info(f'Waiting {DICE_SETTLE_SEC}s for dice to settle...')
        time.sleep(DICE_SETTLE_SEC)

        model = YOLO(DICE_MODEL_PATH)
        frame = self._get_fresh_frame(timeout=5.0)
        if frame is None:
            self.get_logger().error('No camera frame for dice detection!')
            return None

        x1, y1, x2, y2 = DICE_ROI
        roi = frame[y1:y2, x1:x2]
        roi_large = cv2.resize(roi, (roi.shape[1]*DICE_SCALE, roi.shape[0]*DICE_SCALE),
                               interpolation=cv2.INTER_CUBIC)
        results = model(roi_large, conf=0.1, iou=0.3, verbose=False)
        faces = [int(r.cls) + 1 for res in results for r in res.boxes]

        print('\n' + '='*40)
        if len(faces) == 2:
            total   = sum(faces)
            outcome = 'BIG' if total >= 7 else 'SMALL'
            print(f'  Dice: {faces[0]} + {faces[1]} = {total}')
            print(f'  Dice result: {outcome}')
        elif len(faces) == 1:
            print(f'  Only 1 die detected: {faces[0]}')
            outcome = None
        else:
            print('  No dice detected!')
            outcome = None
        print('='*40 + '\n')
        return outcome

    # ── Game outcome ─────────────────────────────────────────────
    def _resolve_outcome(self, dice_outcome):
        """Print the round summary and return 'WIN' / 'LOSE' / None."""
        print('\n' + '='*40)
        print(f'  Player bet:  {self._player_bet}')
        print(f'  Dice result: {dice_outcome}')
        if self._player_bet is None or dice_outcome is None:
            print('  Cannot resolve, missing data')
            print('='*40 + '\n')
            return None
        if self._player_bet == dice_outcome:
            print('  *** PLAYER WINS, PAYOUT ***')
            result = 'WIN'
        else:
            print('  *** PLAYER LOSES, COLLECT ***')
            result = 'LOSE'
        print('='*40 + '\n')
        return result

    # ── Controller switch ────────────────────────────────────────
    def _switch_controllers(self):
        self.get_logger().info('Switching controllers: arm→JTC, gripper→GripperAction...')
        self._switch_client.wait_for_service()
        req = SwitchController.Request()
        req.activate_controllers = [
            'left_arm_controller',      'right_arm_controller',
            'left_gripper_controller',  'right_gripper_controller',
        ]
        req.deactivate_controllers = [
            'left_arm_fwd_controller',      'right_arm_fwd_controller',
            'left_gripper_fwd_controller',  'right_gripper_fwd_controller',
        ]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        future = self._switch_client.call_async(req)
        while not future.done():
            time.sleep(0.05)
        ok = future.result().ok
        self.get_logger().info('Switch OK' if ok else 'Switch FAILED')
        return ok

    # ── Arm / gripper ────────────────────────────────────────────
    def _move_arm(self, arm, row, secs):
        positions = [float(row[j]) for j in JOINT_COLS]
        self._arm_pubs[arm].publish(make_trajectory(ARM_JOINTS[arm], positions, secs))

    def _move_gripper(self, arm, row):
        if 'gripper' not in row:
            return
        client = self._gripper_clients[arm]
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'[{arm}] Gripper action server not available!')
            return
        goal = GripperCommand.Goal()
        goal.command.position   = float(row['gripper'])
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        client.send_goal_async(goal)

    def _run_action(self, action: dict):
        name       = action['name']
        left_csv   = action.get('left')
        right_csv  = action.get('right')
        secs       = float(action.get('secs', 2.0))
        wait_after = float(action.get('wait_after', 0.0))
        read_dice  = action.get('read_dice', False)

        left_rows  = load_rows(left_csv)  if left_csv  else None
        right_rows = load_rows(right_csv) if right_csv else None

        if left_rows and right_rows and len(left_rows) != len(right_rows):
            self.get_logger().error(
                f'[{name}] CSV row mismatch: left={len(left_rows)}, right={len(right_rows)}')
            return False, None

        n_rows = len(left_rows or right_rows)
        self.get_logger().info(f'▶ {name} ({n_rows} rows, {secs}s/row)')

        for i in range(n_rows):
            self.get_logger().info(f'  row {i+1}/{n_rows}')
            if left_rows:
                self._move_arm('left',     left_rows[i],  secs)
                self._move_gripper('left', left_rows[i])
            if right_rows:
                self._move_arm('right',     right_rows[i], secs)
                self._move_gripper('right', right_rows[i])
            time.sleep(secs + 0.3)

        if wait_after > 0:
            time.sleep(wait_after)

        dice_outcome = None
        if read_dice:
            dice_outcome = self._read_dice()

        return True, dice_outcome

    # ── Bringup lifecycle (managed by orchestrator) ──────────────
    def _start_bringup(self):
        """Launch bi_soa_bringup in its own process group and wait until ready."""
        cmd = list(BRINGUP_CMD)
        if self._bringup_args:
            cmd += self._bringup_args.split()
        self.get_logger().info('Starting bringup: ' + ' '.join(cmd))
        self._bringup_proc = subprocess.Popen(cmd, start_new_session=True)
        self._wait_bringup_ready()

    def _wait_bringup_ready(self, cm_timeout=90.0):
        """Best-effort readiness: controller_manager up, joint states + camera publishing."""
        self.get_logger().info('Waiting for bringup to come up...')
        if not self._switch_client.wait_for_service(timeout_sec=cm_timeout):
            self.get_logger().warn('controller_manager switch service not up in time')

        deadline = time.time() + 30.0
        while time.time() < deadline and self.count_publishers(JOINT_STATES_TOPIC) == 0:
            time.sleep(0.5)
        if self.count_publishers(JOINT_STATES_TOPIC) == 0:
            self.get_logger().warn(f'{JOINT_STATES_TOPIC} not publishing yet')

        cam_deadline = time.time() + 20.0
        while time.time() < cam_deadline and self.count_publishers(CAM_TOPIC) == 0:
            time.sleep(0.5)
        if self.count_publishers(CAM_TOPIC) == 0:
            self.get_logger().warn(f'Camera topic {CAM_TOPIC} not publishing yet')

        # Important: wait until arm(JTC)+gripper controllers are truly active before returning,
        # otherwise trajectory/gripper commands are sent but nothing executes them.
        self._wait_controllers_active(JTC_CONTROLLERS, timeout=60.0)

        self.get_logger().info('Bringup ready')
        time.sleep(1.0)

    def _list_controllers(self):
        """Return {name: state} from the controller_manager, or {} on failure."""
        if not self._list_client.wait_for_service(timeout_sec=5.0):
            return {}
        try:
            future = self._list_client.call_async(ListControllers.Request())
            t0 = time.time()
            while not future.done() and time.time() - t0 < 5.0:
                time.sleep(0.05)
            if not future.done() or future.result() is None:
                return {}
            return {c.name: c.state for c in future.result().controller}
        except Exception as e:
            self.get_logger().warn(f'list_controllers failed: {e}')
            return {}

    def _wait_controllers_active(self, names, timeout=60.0):
        """Poll until all `names` report state 'active'. Returns True/False."""
        need = set(names)
        deadline = time.time() + timeout
        while time.time() < deadline:
            states = self._list_controllers()
            active = {n for n, s in states.items() if s == 'active'}
            if need.issubset(active):
                self.get_logger().info(f'Controllers active: {sorted(need)}')
                return True
            time.sleep(0.5)
        missing = need - {n for n, s in self._list_controllers().items() if s == 'active'}
        self.get_logger().warn(f'Controllers not active in time: {sorted(missing)}')
        return False

    def _ensure_jtc_active(self):
        """Make sure arm(JTC)+gripper controllers are active.
        Managed bringup already launches them active (controller:=jtc);
        for an unmanaged forward-mode bringup, fall back to a switch."""
        if self._wait_controllers_active(JTC_CONTROLLERS, timeout=5.0):
            return True
        self.get_logger().info('JTC controllers not active; switching...')
        self._switch_controllers()
        return self._wait_controllers_active(JTC_CONTROLLERS, timeout=15.0)

    def _ensure_bringup(self):
        """Start a managed bringup if we own the lifecycle and none is running."""
        if self._manage_bringup and self._bringup_proc is None:
            self._start_bringup()
        # If unmanaged, assume the user already launched bringup manually.

    def stop_bringup(self):
        """Tear down the managed bringup process group (SIGINT → SIGTERM → SIGKILL)."""
        if self._bringup_proc is None:
            return
        self.get_logger().info('Stopping bringup (SIGINT to process group)...')
        pgid = None
        try:
            pgid = os.getpgid(self._bringup_proc.pid)
        except ProcessLookupError:
            self._bringup_proc = None
            return

        def _signal(sig):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                pass

        _signal(signal.SIGINT)
        try:
            self._bringup_proc.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Bringup ignored SIGINT; escalating to SIGTERM')
            _signal(signal.SIGTERM)
            try:
                self._bringup_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Bringup ignored SIGTERM; SIGKILL')
                _signal(signal.SIGKILL)
        self._bringup_proc = None
        self._wait_ports_free(FOLLOWER_PORTS, timeout=10.0)
        self.get_logger().info('Bringup stopped')

    def _ensure_ros_released(self):
        """Free all USB devices before native lerobot takes over."""
        if self._bringup_proc is not None:
            self.stop_bringup()           # managed: kill whole launch tree
        else:
            self._release_ros_devices()   # unmanaged / stray: pkill holders

    # ── Native lerobot handoff (NO rosetta) ──────────────────────
    def _wait_ports_free(self, ports, timeout=10.0):
        """Block until the given serial ports are no longer held, escalating
        to SIGKILL on stubborn holders. Best-effort if `fuser` is missing."""
        if shutil.which('fuser') is None:
            self.get_logger().warn('`fuser` not found; sleeping 3s instead of verifying ports')
            time.sleep(2.0)
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            busy = []
            for p in ports:
                if not os.path.exists(p):
                    continue
                r = subprocess.run(['fuser', p], capture_output=True)
                if r.returncode == 0:        # someone still holds it
                    busy.append(p)
            if not busy:
                self.get_logger().info(f'Serial ports free: {ports}')
                return
            for p in busy:                    # escalate
                subprocess.run(['fuser', '-k', p], check=False)
            time.sleep(1.0)
        self.get_logger().warn('Timed out waiting for ports to free; continuing anyway')

    def _release_ros_devices(self):
        """Kill the ROS nodes holding USB devices so native lerobot can open them."""
        self.get_logger().info('Releasing ROS-held USB devices for lerobot handoff...')
        for pat in ROS_DEVICE_HOLDERS:
            subprocess.run(['pkill', '-f', pat], check=False)
        time.sleep(2.0)                       # give nodes time to close fds
        self._wait_ports_free(FOLLOWER_PORTS, timeout=10.0)

    def _build_lerobot_cmd(self, cfg):
        """Build the native single-arm ACT inference command.
        Use a timestamped dataset.root each run so lerobot-record does not error on an existing directory."""
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        return [
            'lerobot-record',
            '--robot.type=so101_follower',
            f'--robot.port={ACT_ROBOT_PORT}',
            f'--robot.id={ACT_ROBOT_ID}',
            f'--robot.calibration_dir={ACT_ROBOT_CALIB}',
            '--teleop.type=so101_leader',
            f'--teleop.port={ACT_TELEOP_PORT}',
            f'--teleop.id={ACT_TELEOP_ID}',
            f'--teleop.calibration_dir={ACT_TELEOP_CALIB}',
            f'--robot.cameras={ACT_CAMERAS}',
            f'--dataset.repo_id={cfg["repo_id"]}_{stamp}',
            f'--dataset.root={cfg["dataset_root"]}_{stamp}',
            f'--dataset.num_episodes={cfg.get("episodes", 1)}',
            f'--dataset.episode_time_s={cfg.get("episode_time_s", 60)}',
            f'--dataset.single_task={cfg["task"]}',
            '--dataset.push_to_hub=false',
            f'--policy.path={cfg["policy_path"]}',
            '--display_data=false',
            '--play_sounds=false',
        ]

    def _run_lerobot_policy(self, cfg):
        """Run native lerobot ACT inference as a subprocess; block until done."""
        cmd = self._build_lerobot_cmd(cfg)
        self.get_logger().info('Launching native lerobot inference:')
        self.get_logger().info('  ' + ' '.join(cmd))
        proc = subprocess.run(cmd, env=dict(os.environ))
        if proc.returncode == 0:
            self.get_logger().info('lerobot policy finished OK')
            return True
        self.get_logger().error(f'lerobot exited with code {proc.returncode}')
        return False

    def _run_policy_phase(self, cfg):
        """Full policy phase: (optional home under ROS) → release → lerobot."""
        # Optional: drive to home under ROS first so the ACT start state holds.
        home = cfg.get('home')
        if home:
            self._ensure_bringup()
            if self._switch_controllers():
                time.sleep(1.0)
                self._run_action(home)
        self._ensure_ros_released()
        # ACT uses different serial ports than bringup; additionally confirm they are free.
        self._wait_ports_free(ACT_REQUIRED_PORTS, timeout=10.0)
        ok = self._run_lerobot_policy(cfg)
        if ok and cfg.get('restart_bringup_after', False) and self._manage_bringup:
            self._start_bringup()
        self.get_logger().info(
            '=== Policy phase complete ===' if ok else '=== Policy phase FAILED ===')

    def _run_sequence(self, sequence):
        """Run a list of CSV actions; return (ok, last_dice_outcome)."""
        dice_outcome = None
        for action in sequence:
            ok, outcome = self._run_action(action)
            if not ok:
                self.get_logger().error(f'Aborted at: {action["name"]}')
                return False, dice_outcome
            if outcome is not None:
                dice_outcome = outcome
        return True, dice_outcome

    def _run_one_round(self):
        """One full game round:
        bet → dice (CSV) → resolve → WIN: chip_dispense (CSV, stays in ROS)
                                    → LOSE: chip_collect (lerobot ACT, hard handoff).
        """
        # bringup may have been torn down for lerobot in the previous LOSE branch; restart if needed.
        self._ensure_bringup()
        if not self._ensure_jtc_active():
            self.get_logger().error('JTC/gripper controllers not active; aborting round')
            return
        time.sleep(1.0)

        # 1) Player places a bet (overhead camera). Blocks until a bet is detected.
        self._read_bet()
        if not rclpy.ok():
            return

        # 2) Roll + read the dice (CSV sequence ending in read_dice).
        ok, dice_outcome = self._run_sequence(SEQUENCES['dice_sequence'])
        if not ok:
            return

        # 2.5) Settlement check: is a chip still on the tray? Removed early -> alarm and stop.
        time.sleep(0.5)   # let the arm leave the chip area to avoid occlusion false-positives
        if not self._verify_chip_present():
            self._alarm_chip_removed()
            self._abort = True
            return

        # 3) Resolve the bet against the dice (judged by the locked initial bet).
        result = self._resolve_outcome(dice_outcome)
        if result is None:
            self.get_logger().warn('Missing bet/dice; skipping payout/collect')
            return

        # 4) Branch.
        if result == 'WIN':
            # Dealer pays the player → dispense chips. CSV, no handoff needed.
            self.get_logger().info('WIN → chip_dispense (CSV, under ROS)')
            self._run_sequence(SEQUENCES['chip_dispense'])
        else:
            # Dealer collects the player's chips → lerobot ACT. Hard handoff.
            # this stops bringup to run lerobot; next round _ensure_bringup brings it back up.
            self.get_logger().info('LOSE → chip_collect (native lerobot ACT)')
            self._run_policy_phase(LEROBOT_POLICIES['chip_collect'])

        self.get_logger().info('=== Round complete ===')

    def _run_game_loop(self):
        """Run the game continuously: a round ends -> 10s buffer (player clears chips and re-bets) -> back to waiting for a bet.
        Exits when a chip is removed early (self._abort) or on Ctrl-C."""
        round_idx = 0
        while rclpy.ok() and not self._abort:
            round_idx += 1
            self.get_logger().info(f'\n########## ROUND {round_idx} ##########')
            self._run_one_round()
            if self._abort or not rclpy.ok():
                break
            # 10s inter-round buffer: player clears old chips and re-bets. No detection here.
            self.get_logger().info('Round buffer: 10s for player to clear and re-bet...')
            time.sleep(10.0)
        if self._abort:
            self.get_logger().error('Game loop stopped (chip removed / abort).')

    # ── Main run loop ────────────────────────────────────────────
    def run(self):
        # Continuous orchestrated game loop (single command).
        if self._is_round:
            self._run_game_loop()
            return

        # Native lerobot policy phase: hard handoff, skip bet/dice/JTC switch.
        if self._is_policy_phase:
            self._run_policy_phase(self._policy_cfg)
            return

        # CSV phase needs ROS control + cameras: bring it up if we own it.
        self._ensure_bringup()

        if not self._ensure_jtc_active():
            self.get_logger().error('JTC/gripper controllers not active; aborting')
            return
        time.sleep(1.0)

        self._read_bet()

        ok, dice_outcome = self._run_sequence(self._sequence)
        if not ok:
            return

        self._resolve_outcome(dice_outcome)
        self.get_logger().info('=== Round complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Always tear down a managed bringup so we never leave orphan processes.
        node.stop_bringup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()