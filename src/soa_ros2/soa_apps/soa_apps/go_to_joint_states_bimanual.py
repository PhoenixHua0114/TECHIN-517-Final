#!/usr/bin/env python3
"""
go_to_joint_states_bimanual.py
双臂FK：arm 使用 JointTrajectoryController 实现平滑运动，
gripper 使用 GripperActionController（通过 action 驱动）。
两个CSV按行对齐，每行双臂同时执行。left_csv 或 right_csv 可以为空（单臂模式）。

CSV格式: shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper

Usage:
    ros2 launch soa_bringup bi_soa_bringup.launch.py
    # 双臂
    ros2 run soa_apps go_to_joint_states_bimanual \
        --ros-args \
        -p left_csv:=/home/ubuntu/techin517/craps_left.csv \
        -p right_csv:=/home/ubuntu/techin517/craps_right.csv \
        -p seconds_per_move:=2.0
    # 单臂（只动左臂）
    ros2 run soa_apps go_to_joint_states_bimanual \
        --ros-args \
        -p left_csv:=/home/ubuntu/techin517/craps_left.csv \
        -p seconds_per_move:=2.0
    # 单臂（只动右臂）
    ros2 run soa_apps go_to_joint_states_bimanual \
        --ros-args \
        -p right_csv:=/home/ubuntu/techin517/craps_right.csv \
        -p seconds_per_move:=2.0
"""

import csv
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_COLS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

ARM_JOINTS = {
    'left':  ['left_shoulder_pan',  'left_shoulder_lift',  'left_elbow_flex',  'left_wrist_flex',  'left_wrist_roll'],
    'right': ['right_shoulder_pan', 'right_shoulder_lift', 'right_elbow_flex', 'right_wrist_flex', 'right_wrist_roll'],
}
ARM_TOPIC = {
    'left':  '/follower/left_arm_controller/joint_trajectory',
    'right': '/follower/right_arm_controller/joint_trajectory',
}
# gripper 现在走 GripperActionController 的 action，而不是 fwd controller 的 topic
GRIPPER_ACTION = {
    'left':  '/follower/left_gripper_controller/gripper_cmd',
    'right': '/follower/right_gripper_controller/gripper_cmd',
}


def load_rows(path: str) -> list:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def make_trajectory(joint_names: list, positions: list, secs: float) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = joint_names
    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    msg.points = [pt]
    return msg


class BimanualFK(Node):

    def __init__(self):
        super().__init__('go_to_joint_states_bimanual')
        self.declare_parameter('left_csv',         '')
        self.declare_parameter('right_csv',        '')
        self.declare_parameter('seconds_per_move', 2.0)
        self.declare_parameter('gripper_max_effort', 5.0)

        self._arm_pubs = {
            arm: self.create_publisher(JointTrajectory, topic, 10)
            for arm, topic in ARM_TOPIC.items()
        }
        # gripper action clients
        self._gripper_clients = {
            arm: ActionClient(self, GripperCommand, action_name)
            for arm, action_name in GRIPPER_ACTION.items()
        }
        self._switch_client = self.create_client(
            SwitchController,
            '/follower/controller_manager/switch_controller'
        )

    def _switch_controllers(self):
        self.get_logger().info('Switching controllers...')
        self._switch_client.wait_for_service()
        req = SwitchController.Request()
        # arm 用 trajectory controller，gripper 用 action controller
        req.activate_controllers = [
            'left_arm_controller',     'right_arm_controller',
            'left_gripper_controller', 'right_gripper_controller',
        ]
        # 停掉 fwd controller（arm + gripper），避免抢占同一命令接口
        req.deactivate_controllers = [
            'left_arm_fwd_controller',     'right_arm_fwd_controller',
            'left_gripper_fwd_controller', 'right_gripper_fwd_controller',
        ]
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self._switch_client.call_async(req)
        while not future.done():
            time.sleep(0.05)
        if future.result().ok:
            self.get_logger().info('Controller switch successful')
        else:
            self.get_logger().error('Controller switch failed!')

    def _move_arm(self, arm: str, row: dict, secs: float):
        positions = [float(row[j]) for j in JOINT_COLS]
        self._arm_pubs[arm].publish(make_trajectory(ARM_JOINTS[arm], positions, secs))

    def _send_gripper_goal(self, arm: str, row: dict):
        """发送 gripper action 目标（异步），返回 goal_handle future。"""
        max_effort = self.get_parameter('gripper_max_effort').get_parameter_value().double_value
        goal = GripperCommand.Goal()
        goal.command.position = float(row['gripper'])
        goal.command.max_effort = max_effort

        client = self._gripper_clients[arm]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{arm} gripper action server not available')
            return None
        return client.send_goal_async(goal)

    def _wait_goal_handle(self, future):
        """等待 send_goal_async 的 future 完成，返回 goal_handle。executor 在后台线程 spin。"""
        if future is None:
            return None
        while not future.done():
            time.sleep(0.02)
        return future.result()

    def run(self):
        left_csv  = self.get_parameter('left_csv').get_parameter_value().string_value
        right_csv = self.get_parameter('right_csv').get_parameter_value().string_value
        secs      = self.get_parameter('seconds_per_move').get_parameter_value().double_value

        # 支持单臂模式：left_csv 或 right_csv 可以为空
        left_rows  = load_rows(left_csv)  if left_csv  else None
        right_rows = load_rows(right_csv) if right_csv else None

        if left_rows is None and right_rows is None:
            self.get_logger().error('Both left_csv and right_csv are empty! At least one is required.')
            return

        # 双臂模式下检查行数一致
        if left_rows and right_rows and len(left_rows) != len(right_rows):
            self.get_logger().error(
                f'CSV行数不一致: left={len(left_rows)}, right={len(right_rows)}'
            )
            return

        n_rows = len(left_rows or right_rows)
        active_arms = []
        if left_rows:  active_arms.append('left')
        if right_rows: active_arms.append('right')
        self.get_logger().info(f'Active arms: {active_arms}, {n_rows} rows, seconds_per_move={secs}')

        # 切换 controller
        self._switch_controllers()
        time.sleep(0.5)
        time.sleep(1.0)

        for i in range(n_rows):
            self.get_logger().info(f'=== Row {i+1}/{n_rows} ===')

            # 1) 先发 arm trajectory（双臂同时）
            if left_rows:
                self._move_arm('left',  left_rows[i],  secs)
            if right_rows:
                self._move_arm('right', right_rows[i], secs)

            # 2) 同时发 gripper action goal（异步），双臂并行
            gripper_futures = {}
            if left_rows:
                gripper_futures['left']  = self._send_gripper_goal('left',  left_rows[i])
            if right_rows:
                gripper_futures['right'] = self._send_gripper_goal('right', right_rows[i])

            # 3) 确认 gripper goal 被接收（可选，便于排错）
            for arm, gf in gripper_futures.items():
                handle = self._wait_goal_handle(gf)
                if handle is None or not handle.accepted:
                    self.get_logger().warn(f'{arm} gripper goal not accepted')

            # 等待 arm 运动 + gripper 动作完成
            time.sleep(secs + 0.5)

        self.get_logger().info('=== Sequence complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = BimanualFK()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()