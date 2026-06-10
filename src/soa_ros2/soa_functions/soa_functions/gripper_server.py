#!/usr/bin/env python3
"""
Gripper action server for the SOA arm.

Uses pymoveit2 MoveIt2 to control the gripper joint position via the gripper
JointTrajectoryController (FollowJointTrajectory action interface).

Usage:
    ros2 run soa_functions gripper_server
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State

from soa_interfaces.action import Gripper
from soa_functions import soa_robot


class GripperServer(Node):

    def __init__(self):
        super().__init__('gripper_server')

        self.declare_parameter('prefix', '')

        # prefix = ''       → 单臂模式 (group: gripper)
        # prefix = 'left_'  → 左臂     (group: left_gripper)
        # prefix = 'right_' → 右臂     (group: right_gripper)
        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        if prefix:
            group_name = prefix.rstrip('_') + '_gripper'  # 'left_gripper' / 'right_gripper'
        else:
            group_name = soa_robot.MOVE_GROUP_GRIPPER      # 'gripper'
        self._prefix = prefix

        # Callback group for pymoveit2 (must be reentrant)
        self._cb_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 for the gripper group
        self._moveit2 = MoveIt2(
            node=self,
            joint_names=soa_robot.gripper_joint_names(prefix),
            base_link_name=soa_robot.base_link_name(),
            end_effector_name=soa_robot.end_effector_name(prefix),
            group_name=group_name,
            callback_group=self._cb_group,
        )

        # Create action server
        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper_command',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Gripper action server ready')

    def _wait_until_executed(self):
        """Wait for MoveIt2 execution without rclpy.spin_once() conflict."""
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            time.sleep(0.1)
        return self._moveit2.motion_suceeded

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Received Gripper goal')

        target_position = goal_handle.request.target_position
        self.get_logger().info(f'Target gripper position: {target_position}')

        result = Gripper.Result()

        # Publish initial feedback
        feedback = Gripper.Feedback()
        feedback.current_position = target_position
        goal_handle.publish_feedback(feedback)

        # Plan asynchronously (avoids rclpy.spin_once() inside executor callback)
        future = self._moveit2.plan_async(
            joint_positions=[target_position],
            joint_names=soa_robot.gripper_joint_names(self._prefix),
        )
        if future is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed: could not get plan future'
            self.get_logger().error(result.message)
            return result

        while not future.done():
            time.sleep(0.1)

        trajectory = self._moveit2.get_trajectory(future)
        if trajectory is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed: no trajectory returned'
            self.get_logger().error(result.message)
            return result

        self._moveit2.execute(trajectory)
        success = self._wait_until_executed()

        # Publish final feedback
        feedback.current_position = target_position
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = success
        result.message = f'Gripper moved to position {target_position:.4f}'
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)

    node = GripperServer()

    # Use MultiThreadedExecutor for pymoveit2 concurrent callbacks
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    # Wait for initialization
    time.sleep(1.0)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()