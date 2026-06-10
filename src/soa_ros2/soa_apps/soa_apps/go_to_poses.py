"""Go to poses app.

Loads saved poses from a CSV file and executes a sequence of
pose and gripper commands to pick up the ArUco cube.

Usage:
    ros2 run soa_apps go_to_poses --ros-args -p csv_path:=/path/to/poses.csv
"""

import csv
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from soa_interfaces.action import MoveToPose, Gripper

# Gripper states
GRIPPER_OPEN   = 1.7453
GRIPPER_CLOSED = 0.1

# Edit this sequence to match your saved poses
SEQUENCE = [
    ('pose',    {'pose_index': 0}),                  # 起点
    ('pose',    {'pose_index': 1}),                  # 抓取位置
    ('gripper', {'position': GRIPPER_CLOSED}),       # 爪子闭合
    ('pose',    {'pose_index': 3}),                  # 抬起来
    ('pose',    {'pose_index': 4}),                  # 移动到盘子正上方
    ('pose',    {'pose_index': 5}),                  # 下降
    ('gripper', {'position': GRIPPER_OPEN}),         # 松开
    ('pose',    {'pose_index': 7}),                  # end pose
]


def load_poses(path: str) -> list:
    """Load saved poses from a CSV file into a list."""
    poses = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            p = Pose()
            p.position.x    = float(row['x'])
            p.position.y    = float(row['y'])
            p.position.z    = float(row['z'])
            p.orientation.x = float(row['qx'])
            p.orientation.y = float(row['qy'])
            p.orientation.z = float(row['qz'])
            p.orientation.w = float(row['qw'])
            poses.append(p)
    return poses


class GoToPoses(Node):

    def __init__(self):
        super().__init__('go_to_poses')

        self._cb_group = ReentrantCallbackGroup()

        # Declare csv file path as a ROS parameter
        self.declare_parameter('csv_path', '')

        # Initialize MoveToPose action client
        self._pose_client = ActionClient(
            self, MoveToPose, 'move_to_pose',
            callback_group=self._cb_group,
        )

        # Initialize Gripper action client
        self._gripper_client = ActionClient(
            self, Gripper, 'gripper_command',
            callback_group=self._cb_group,
        )

    def send_pose_goal(self, pose: Pose) -> bool:
        """Use the MoveToPose action client to move the arm."""
        self._pose_client.wait_for_server()
        goal = MoveToPose.Goal()
        goal.target_pose = pose
        future = self._pose_client.send_goal_async(
            goal,
            feedback_callback=self._pose_feedback_callback,
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pose goal rejected.')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'Pose result: {result.message}')
        return result.success

    def send_gripper_goal(self, target_position: float) -> bool:
        """Use the Gripper action client to move the gripper."""
        self._gripper_client.wait_for_server()
        goal = Gripper.Goal()
        goal.target_position = target_position
        future = self._gripper_client.send_goal_async(
            goal,
            feedback_callback=self._gripper_feedback_callback,
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper command done.')
        return True

    def _pose_feedback_callback(self, feedback_msg):
        """Handle feedback from the inverse kinematic action server."""
        d = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(f'  Distance to goal: {d:.4f}')

    def _gripper_feedback_callback(self, feedback_msg):
        """Handle feedback from the gripper action server."""
        pos = feedback_msg.feedback.current_position
        self.get_logger().info(f'  Gripper position: {pos:.4f}')

    def run(self):
        """Load saved poses and execute the sequence."""
        csv_path = (
            self.get_parameter('csv_path')
            .get_parameter_value().string_value
        )
        if not csv_path:
            self.get_logger().error('No csv_path parameter provided.')
            return

        poses = load_poses(csv_path)
        self.get_logger().info(f'Loaded {len(poses)} poses from {csv_path}')

        for step_type, params in SEQUENCE:
            if step_type == 'pose':
                idx = params['pose_index']
                self.get_logger().info(f'Moving to pose index {idx}...')
                success = self.send_pose_goal(poses[idx])
                if not success:
                    self.get_logger().error(f'Failed at pose {idx}, aborting.')
                    return
            elif step_type == 'gripper':
                pos = params['position']
                label = 'OPEN' if pos > 1.0 else 'CLOSED'
                self.get_logger().info(f'Gripper -> {label}')
                self.send_gripper_goal(pos)

        self.get_logger().info('Sequence complete!')


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoses()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()