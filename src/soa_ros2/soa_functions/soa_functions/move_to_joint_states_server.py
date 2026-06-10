#!/usr/bin/env python3
"""
MoveToJointStates action server for the SOA 5-DOF arm.

Uses pymoveit2 to plan and execute joint-space motion to a target configuration.
Joint names may be specified in the goal; if omitted, all 5 arm joints are assumed
in order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.

Usage:
    ros2 run soa_functions move_to_joint_states_server
"""

import time
from threading import Thread

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State

from soa_interfaces.action import MoveToJointStates
from soa_functions import soa_robot


# Joint limits from soa_description/urdf/soa_macro.xacro
_JOINT_LIMITS = {
    'shoulder_pan':  (-2.04786, 2.04786),
    'shoulder_lift': (-1.89089, 1.89089),
    'elbow_flex':    (-1.69021, 1.69021),
    'wrist_flex':    (-1.78913, 1.78913),
    'wrist_roll':    (-2.99306, 2.99306),
}


class MoveToJointStatesServer(Node):

    def __init__(self):
        super().__init__('move_to_joint_states_server')

        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('allowed_planning_time', 3.0)
        self.declare_parameter('prefix', '')

        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        # prefix = '' → 单臂模式 (group: arm)
        # prefix = 'left_' → 左臂 (group: left_arm)
        # prefix = 'right_' → 右臂 (group: right_arm)
        if prefix:
            group_name = prefix.rstrip('_') + '_arm'   # 'left_arm' / 'right_arm'
        else:
            group_name = soa_robot.MOVE_GROUP_ARM       # 'arm'

        self._cb_group = ReentrantCallbackGroup()

        self._moveit2 = MoveIt2(
            node=self,
            joint_names=soa_robot.joint_names(prefix),
            base_link_name=soa_robot.base_link_name(),
            end_effector_name=soa_robot.end_effector_name(prefix),
            group_name=group_name,
            callback_group=self._cb_group,
        )

        self._moveit2.max_velocity = (
            self.get_parameter('max_velocity').get_parameter_value().double_value
        )
        self._moveit2.max_acceleration = (
            self.get_parameter('max_acceleration').get_parameter_value().double_value
        )
        self._moveit2.num_planning_attempts = (
            self.get_parameter('num_planning_attempts')
            .get_parameter_value().integer_value
        )
        self._moveit2.allowed_planning_time = (
            self.get_parameter('allowed_planning_time')
            .get_parameter_value().double_value
        )

        self._action_server = ActionServer(
            self,
            MoveToJointStates,
            'move_to_joint_states',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('MoveToJointStates action server ready')

    def _wait_and_publish_feedback(self, goal_handle, joint_names, target_positions):
        while self._moveit2.query_state() != MoveIt2State.IDLE:
            self._publish_feedback(goal_handle, joint_names, target_positions)
            time.sleep(0.1)
        self._publish_feedback(goal_handle, joint_names, target_positions)
        return self._moveit2.motion_suceeded

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Received MoveToJointStates goal')

        joint_positions = list(goal_handle.request.joint_positions)
        joint_names = list(goal_handle.request.joint_names)

        if not joint_names:
            prefix = self.get_parameter('prefix').get_parameter_value().string_value
            joint_names = soa_robot.joint_names(prefix)

        result = MoveToJointStates.Result()

        # --- Validation: length ---
        if len(joint_positions) != len(joint_names):
            goal_handle.abort()
            result.success = False
            result.message = f'Length mismatch: {len(joint_names)} names vs {len(joint_positions)} positions'
            self.get_logger().warn(result.message)
            return result
        
        prefix = self.get_parameter('prefix').get_parameter_value().string_value

        # --- Validation: joint limits ---
        for name, pos in zip(joint_names, joint_positions):
            bare_name = name.removeprefix(prefix)   # 'left_shoulder_pan' → 'shoulder_pan'
            if bare_name not in _JOINT_LIMITS:
                goal_handle.abort()
                result.success = False
                result.message = f'Unknown joint name: {name}'
                self.get_logger().warn(result.message)
                return result
            lo, hi = _JOINT_LIMITS[bare_name]
            if not (lo <= pos <= hi):
                goal_handle.abort()
                result.success = False
                result.message = f'Joint {name}={pos:.4f} out of limits [{lo}, {hi}]'
                self.get_logger().warn(result.message)
                return result

        self.get_logger().info(
            f'Target joints: {joint_names}, positions: {joint_positions}'
        )

        planning_time = (
            self.get_parameter('allowed_planning_time')
            .get_parameter_value().double_value
        )
        self._moveit2.allowed_planning_time = planning_time
        self._moveit2.clear_goal_constraints()

        future = self._moveit2.plan_async(
            joint_positions=joint_positions,
            joint_names=joint_names,
            start_joint_state=self._moveit2.joint_state,
        )

        if future is None:
            goal_handle.abort()
            result.success = False
            result.message = 'plan_async() returned None'
            self.get_logger().error(result.message)
            return result

        while not future.done():
            time.sleep(0.1)

        trajectory = self._moveit2.get_trajectory(future)
        if trajectory is None:
            goal_handle.abort()
            result.success = False
            result.message = 'Planning failed: no trajectory found'
            self.get_logger().error(result.message)
            return result

        self._moveit2.execute(trajectory)
        success = self._wait_and_publish_feedback(goal_handle, joint_names, joint_positions)

        self._publish_feedback(goal_handle, joint_names, joint_positions)

        if success:
            goal_handle.succeed()
            result.success = True
            result.message = 'Reached target joint configuration'
            self.get_logger().info(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = 'Execution failed'
            self.get_logger().error(result.message)

        return result

    def _publish_feedback(self, goal_handle, joint_names, target_positions):
        feedback = MoveToJointStates.Feedback()
        try:
            js = self._moveit2.joint_state
            if js is not None and js.name:
                name_to_pos = dict(zip(js.name, js.position))
                errors = [
                    abs(name_to_pos[n] - t)
                    for n, t in zip(joint_names, target_positions)
                    if n in name_to_pos
                ]
                feedback.max_joint_error = max(errors) if errors else -1.0
            else:
                feedback.max_joint_error = -1.0
        except Exception:
            feedback.max_joint_error = -1.0
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToJointStatesServer()

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    time.sleep(1.0)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()