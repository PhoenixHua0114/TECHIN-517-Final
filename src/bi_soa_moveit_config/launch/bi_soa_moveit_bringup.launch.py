"""Launch MoveIt with the real bimanual SOA follower arms.

Hardware (ros2_control + controllers) runs in the global namespace so MoveIt's default
action client paths (/left_arm_controller/follow_joint_trajectory, etc.)
match the server paths directly — no remapping required.

A dedicated MoveIt RSP (no frame_prefix) subscribes to /joint_states published by the
global-namespace joint_state_broadcaster, and publishes TF frames matching the SRDF
virtual joint definition.

Usage:
    ros2 launch bi_soa_moveit_config bi_soa_moveit_bringup.launch.py
    ros2 launch bi_soa_moveit_config bi_soa_moveit_bringup.launch.py cameras:=true
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


def _calib_xacro_args(prefix: str, calib: dict) -> list:
    """Return flat list of xacro 'key:=value' strings for one arm's calibration data."""
    args = []
    for j in JOINTS:
        args += [
            f' {prefix}{j}_id:=', str(calib[j]['id']),
            f' {prefix}{j}_offset:=', str(calib[j]['offset']),
        ]
    return args


def launch_setup(context, *args, **kwargs):
    cameras = context.launch_configurations['cameras']

    moveit_config = (
        MoveItConfigsBuilder('bi_soa', package_name='bi_soa_moveit_config')
        .to_moveit_configs()
    )
    launch_pkg = moveit_config.package_path

    bringup_share = get_package_share_directory('soa_bringup')
    description_share = get_package_share_directory('soa_description')
    moveit_share = get_package_share_directory('bi_soa_moveit_config')

    # Load hardware parameters (serial ports, calibration dirs) from bi_soa_params.yaml
    params_file = os.path.join(bringup_share, 'config', 'bi_soa_params.yaml')
    with open(params_file) as f:
        hw = yaml.safe_load(f)['/**']['ros__parameters']
    lf_params = hw['left_follower']
    rf_params = hw['right_follower']

    # Load per-joint calibration offsets and servo IDs for both arms
    lf_calib = load_arm_calibration(lf_params['calibration_dir'], lf_params['id'])
    rf_calib = load_arm_calibration(rf_params['calibration_dir'], rf_params['id'])

    # Generate URDF with correct serial ports and calibration for both arms
    xacro_file = os.path.join(description_share, 'urdf', 'bi_soa.urdf.xacro')
    urdf_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' left_usb_port:=', lf_params['usb_port'],
        ' right_usb_port:=', rf_params['usb_port'],
        ' left_leader_mode:=false',
        ' right_leader_mode:=false',
        ' use_sim:=false',
        *_calib_xacro_args('left_', lf_calib),
        *_calib_xacro_args('right_', rf_calib),
    ])
    hw_robot_description = {
        'robot_description': ParameterValue(urdf_cmd, value_type=str)
    }

    # ros2_control_node in GLOBAL namespace.
    # Controllers land at /left_arm_controller, /right_arm_controller, etc. —
    # exactly where moveit_simple_controller_manager looks by default.
    controllers_yaml = os.path.join(moveit_share, 'config', 'moveit_controllers_hw.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[hw_robot_description, controllers_yaml],
    )

    # Spawn controllers sequentially to avoid race conditions on startup.
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    spawner_left_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller'],
        output='screen',
    )
    spawner_right_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller'],
        output='screen',
    )
    spawner_left_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller'],
        output='screen',
    )
    spawner_right_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller'],
        output='screen',
    )

    # MoveIt RSP: subscribes to /joint_states directly (no remapping needed).
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
        RegisterEventHandler(OnProcessExit(target_action=spawner_jsb, on_exit=[spawner_left_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawner_left_arm, on_exit=[spawner_right_arm])),
        RegisterEventHandler(OnProcessExit(target_action=spawner_right_arm, on_exit=[spawner_left_gripper])),
        RegisterEventHandler(OnProcessExit(target_action=spawner_left_gripper, on_exit=[spawner_right_gripper])),
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
            launch_arguments={'config_file': 'bi_soa_cameras.yaml'}.items(),
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
