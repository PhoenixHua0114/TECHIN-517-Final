"""Launch MoveIt with the real SOA follower arm.

Hardware (ros2_control + controllers) runs in the global namespace so MoveIt's default
action client paths (/arm_controller/follow_joint_trajectory, /gripper_controller/gripper_cmd)
match the server paths directly — no remapping required.

A dedicated MoveIt RSP (no frame_prefix) subscribes to /joint_states published by the
global-namespace joint_state_broadcaster, and publishes base_link, shoulder_link, …
TF frames matching the SRDF virtual joint definition.

Usage:
    ros2 launch soa_moveit_config soa_moveit_bringup.launch.py
    ros2 launch soa_moveit_config soa_moveit_bringup.launch.py cameras:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from soa_bringup.calibration_loader import load_arm_calibration

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def _calib_xacro_args(calib: dict) -> list:
    """Return flat list of xacro 'key:=value' strings for calibration data."""
    args = []
    for j in JOINTS:
        args += [f' {j}_id:=', str(calib[j]['id']),
                 f' {j}_offset:=', str(calib[j]['offset'])]
    return args


def launch_setup(context, *args, **kwargs):
    cameras = context.launch_configurations['cameras']

    moveit_config = (
        MoveItConfigsBuilder('soa', package_name='soa_moveit_config')
        .to_moveit_configs()
    )
    launch_pkg = moveit_config.package_path

    bringup_share = get_package_share_directory('soa_bringup')
    description_share = get_package_share_directory('soa_description')
    moveit_share = get_package_share_directory('soa_moveit_config')

    # Load hardware parameters (serial port, calibration dir) from soa_params.yaml
    params_file = os.path.join(bringup_share, 'config', 'soa_params.yaml')
    with open(params_file) as f:
        hw = yaml.safe_load(f)['/**']['ros__parameters']
    follower_params = hw['follower']

    # Load per-joint calibration offsets and servo IDs
    follower_calib = load_arm_calibration(
        follower_params['calibration_dir'],
        follower_params['id'],
    )

    # Generate URDF with correct serial port and calibration (needed by ros2_control hardware)
    xacro_file = os.path.join(description_share, 'urdf', 'soa.urdf.xacro')
    follower_urdf_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' usb_port:=', follower_params['usb_port'],
        ' leader_mode:=false',
        ' use_sim:=false',
        *_calib_xacro_args(follower_calib),
    ])
    hw_robot_description = {
        'robot_description': ParameterValue(follower_urdf_cmd, value_type=str)
    }

    # ros2_control_node in GLOBAL namespace.
    # Controllers land at /arm_controller and /gripper_controller — exactly where
    # moveit_simple_controller_manager looks by default (no remapping needed).
    controllers_yaml = os.path.join(moveit_share, 'config', 'moveit_controllers_hw.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[hw_robot_description, controllers_yaml],
    )

    # Spawn controllers sequentially to avoid race conditions on startup.
    # joint_state_broadcaster publishes to /joint_states (global namespace).
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )
    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    # MoveIt RSP: subscribes to /joint_states directly (no remapping needed).
    # Publishes base_link, shoulder_link, … TF frames matching the SRDF.
    moveit_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # Static TF: world -> base_link (from SRDF virtual_joint definition)
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_pkg / 'launch/static_virtual_joint_tfs.launch.py')
        ),
    )

    # move_group: no remappings — controllers are in global namespace.
    # moveit_controllers.yaml lists arm_controller / gripper_controller (no prefix),
    # which resolves to /arm_controller/follow_joint_trajectory etc. directly.
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
        ],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
    )

    # RViz with MoveIt MotionPlanning plugin
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_pkg / 'launch/moveit_rviz.launch.py')
        ),
    )

    actions = [
        ros2_control_node,
        spawner_jsb,
        RegisterEventHandler(OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawner_arm, on_exit=[spawner_gripper])),
        moveit_rsp,
        static_tf,
        move_group,
        rviz,
    ]

    if cameras == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'include', 'cameras.launch.py')
            ),
            launch_arguments={'config_file': 'soa_cameras.yaml'}.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras',
            default_value='true',
            description='Launch camera nodes alongside the arm hardware.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
