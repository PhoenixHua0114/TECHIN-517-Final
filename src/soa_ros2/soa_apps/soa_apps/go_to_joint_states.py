#!/usr/bin/env python3
"""...(保留原有 docstring)"""

import csv

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from soa_interfaces.action import Gripper, MoveToJointStates  # TODO 填这里


DEFAULT_CSV_PATH = '/home/ubuntu/techin517/soa_ws/joints.csv'
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def load_rows(path: str) -> list:
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        return list(reader)


class GoToJointStates(Node):

    def __init__(self):
        super().__init__('go_to_joint_states')
        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)

        self._joint_client = ActionClient(self, MoveToJointStates, 'move_to_joint_states')
        self._gripper_client = ActionClient(self, Gripper, 'gripper_command')

    def send_joint_goal(self, joint_positions: list, joint_names: list, label: str = '') -> bool:
        goal = MoveToJointStates.Goal()
        goal.joint_positions = joint_positions
        goal.joint_names = joint_names

        self.get_logger().info(
            f'Sending joint goal ({label}): '
            + ', '.join(f'{n}={p:.4f}' for n, p in zip(joint_names, joint_positions))
        )

        self._joint_client.wait_for_server()
        future = self._joint_client.send_goal_async(
            goal, feedback_callback=self._joint_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Joint goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Joint goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Joint goal failed: {result.message}')
        return result.success

    def send_gripper_goal(self, target_position: float, label: str = '') -> bool:
        goal = Gripper.Goal()
        goal.target_position = target_position

        self.get_logger().info(f'Sending gripper goal ({label}): position={target_position:.4f}')

        self._gripper_client.wait_for_server()
        future = self._gripper_client.send_goal_async(
            goal, feedback_callback=self._gripper_feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Gripper goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Gripper goal failed: {result.message}')
        return result.success

    def _joint_feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Joint feedback: max_joint_error={feedback_msg.feedback.max_joint_error:.4f}'
        )

    def _gripper_feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f'Gripper feedback: current_position={feedback_msg.feedback.current_position:.4f}'
        )

    def run(self, csv_path):
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # skip joint name header
            for i, row in enumerate(reader):
                positions = [float(v) for v in row]
                self.get_logger().info(f'Moving to pose {i+1}: {positions}')
                
                req = MoveToJointStates.Request()
                req.joint_names = header
                req.positions = positions
                
                future = self.client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                self.get_logger().info(f'Pose {i+1} complete')
                time.sleep(0.5)  # brief pause between poses