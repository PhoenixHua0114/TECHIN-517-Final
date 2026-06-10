"""Bimanual SOA arm bringup launch file.

Supports four modes depending on launch arguments:
  1. Bi follower only (default):
       ros2 launch soa_bringup bi_soa_bringup.launch.py
  2. Bi leader + bi follower (both pairs overlap in RViz for teleop monitoring):
       ros2 launch soa_bringup bi_soa_bringup.launch.py leader:=true
  3. Bi follower in Gazebo (pending soa_sim development):
       ros2 launch soa_bringup bi_soa_bringup.launch.py use_sim:=true
  4. Bi follower in Gazebo + bi leader in real life:
       ros2 launch soa_bringup bi_soa_bringup.launch.py leader:=true use_sim:=true

Configuration:
  Edit soa_bringup/config/bi_soa_params.yaml to set USB ports, robot IDs,
  and calibration directory paths for all arms.

Namespace and prefix conventions:
  - left_/right_ are joint name PREFIXES (in the URDF joint names)
  - follower/ and leader/ are ROS2 node NAMESPACES

With leader:=true, left_follower and left_leader arms overlap in RViz
(and similarly for right), allowing the operator to see teleoperation tracking.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from soa_bringup.calibration_loader import load_arm_calibration

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def _calib_xacro_args(prefix: str, calib: dict) -> list:
    """Return flat list of xacro ' key:=value' strings for one arm.

    Args:
        prefix: Xacro arg name prefix (e.g. 'left_', 'right_').
        calib:  Dict from load_arm_calibration: {joint: {id, offset}}.
    """
    args = []
    for j in JOINTS:
        args += [
            f' {prefix}{j}_id:=',     str(calib[j]['id']),
            f' {prefix}{j}_offset:=',  str(calib[j]['offset']),
        ]
    return args


def _make_xacro_cmd(xacro_file: str, left_port: str, right_port: str,
                    left_leader_mode: str, right_leader_mode: str,
                    use_sim: str, left_calib: dict, right_calib: dict) -> Command:
    """Build a xacro Command substitution for bi_soa.urdf.xacro."""
    return Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' left_usb_port:=', left_port,
        ' right_usb_port:=', right_port,
        ' left_leader_mode:=', left_leader_mode,
        ' right_leader_mode:=', right_leader_mode,
        ' use_sim:=', use_sim,
        *_calib_xacro_args('left_', left_calib),
        *_calib_xacro_args('right_', right_calib),
    ])


def _make_spawner_chain(controllers: list, cm_namespace: str, inactive: set = None) -> list:
    """Build a sequenced list of controller spawner nodes.

    Controllers are spawned one after another using OnProcessExit handlers
    to avoid race conditions on startup.

    Args:
        controllers: List of controller names to spawn in order.
        cm_namespace: Namespace of the controller_manager to target.
        inactive: Set of controller names to spawn in inactive state.
    """
    inactive = inactive or set()
    spawners = []
    for name in controllers:
        args = [name, '--controller-manager', f'/{cm_namespace}/controller_manager']
        if name in inactive:
            args.append('--inactive')
        spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=args,
            output='screen',
        ))

    actions = [spawners[0]]
    for i in range(1, len(spawners)):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawners[i - 1],
                    on_exit=[spawners[i]],
                )
            )
        )
    return actions


def launch_setup(context, *args, **kwargs):
    use_sim = context.launch_configurations['use_sim']
    leader = context.launch_configurations['leader']
    display = context.launch_configurations['display']
    cameras = context.launch_configurations['cameras']
    controller_mode = context.launch_configurations['controller']

    bringup_share = get_package_share_directory('soa_bringup')
    description_share = get_package_share_directory('soa_description')

    # Load hardware parameters
    params_file = os.path.join(bringup_share, 'config', 'bi_soa_params.yaml')
    with open(params_file) as f:
        hw = yaml.safe_load(f)['/**']['ros__parameters']

    lf_params = hw['left_follower']
    rf_params = hw['right_follower']
    ll_params = hw['left_leader']
    rl_params = hw['right_leader']

    xacro_file = os.path.join(description_share, 'urdf', 'bi_soa.urdf.xacro')
    follower_controllers_yaml = os.path.join(
        description_share, 'config', 'bi_follower_controllers.yaml'
    )
    leader_controllers_yaml = os.path.join(
        description_share, 'config', 'leader_controllers.yaml'
    )

    # Load follower calibrations (always needed)
    try:
        lf_calib = load_arm_calibration(lf_params['calibration_dir'], lf_params['id'])
    except FileNotFoundError as e:
        raise RuntimeError(
            f'[bi_soa_bringup] Left follower calibration not found.\n{e}'
        )
    try:
        rf_calib = load_arm_calibration(rf_params['calibration_dir'], rf_params['id'])
    except FileNotFoundError as e:
        raise RuntimeError(
            f'[bi_soa_bringup] Right follower calibration not found.\n{e}'
        )

    # --- Follower nodes (always launched) ---
    follower_urdf_cmd = _make_xacro_cmd(
        xacro_file=xacro_file,
        left_port=lf_params['usb_port'],
        right_port=rf_params['usb_port'],
        left_leader_mode='false',
        right_leader_mode='false',
        use_sim=use_sim,
        left_calib=lf_calib,
        right_calib=rf_calib,
    )
    follower_robot_description = {'robot_description': ParameterValue(follower_urdf_cmd, value_type=str)}

    follower_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='follower',
        output='screen',
        parameters=[follower_robot_description, {'frame_prefix': 'follower/'}],
    )

    follower_cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='follower',
        output='screen',
        parameters=[follower_robot_description, follower_controllers_yaml],
    )

    if controller_mode == 'forward':
        active = [
            'left_arm_fwd_controller', 'right_arm_fwd_controller',
            'left_gripper_fwd_controller', 'right_gripper_fwd_controller',
        ]
        inactive = {
            'left_arm_controller', 'right_arm_controller',
            'left_gripper_controller', 'right_gripper_controller',
        }
    else:
        active = [
            'left_arm_controller', 'right_arm_controller',
            'left_gripper_controller', 'right_gripper_controller',
        ]
        inactive = {
            'left_arm_fwd_controller', 'right_arm_fwd_controller',
            'left_gripper_fwd_controller', 'right_gripper_fwd_controller',
        }

    follower_spawners = _make_spawner_chain(
        ['joint_state_broadcaster'] + active + list(inactive),
        cm_namespace='follower',
        inactive=inactive,
    )

    all_actions = [follower_rsp, follower_cm] + follower_spawners

    # --- Leader nodes (only when leader:=true) ---
    if leader == 'true':
        try:
            ll_calib = load_arm_calibration(ll_params['calibration_dir'], ll_params['id'])
        except FileNotFoundError as e:
            raise RuntimeError(
                f'[bi_soa_bringup] Left leader calibration not found.\n{e}'
            )
        try:
            rl_calib = load_arm_calibration(rl_params['calibration_dir'], rl_params['id'])
        except FileNotFoundError as e:
            raise RuntimeError(
                f'[bi_soa_bringup] Right leader calibration not found.\n{e}'
            )

        leader_urdf_cmd = _make_xacro_cmd(
            xacro_file=xacro_file,
            left_port=ll_params['usb_port'],
            right_port=rl_params['usb_port'],
            left_leader_mode='true',
            right_leader_mode='true',
            use_sim='false',  # leaders are always real hardware
            left_calib=ll_calib,
            right_calib=rl_calib,
        )
        leader_robot_description = {'robot_description': ParameterValue(leader_urdf_cmd, value_type=str)}

        leader_rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='leader',
            output='screen',
            parameters=[leader_robot_description, {'frame_prefix': 'leader/'}],
        )

        leader_cm = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='leader',
            output='screen',
            parameters=[leader_robot_description, leader_controllers_yaml],
        )

        leader_spawners = _make_spawner_chain(
            ['joint_state_broadcaster'],
            cm_namespace='leader',
        )

        # Static transforms anchoring both arms to 'world' at the same origin.
        # follower_base_link and leader_base_link both at world origin so
        # follower and leader arms overlap (left_ at +Y, right_ at -Y per URDF).
        follower_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='follower_world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'follower/base_link'],
            output='screen',
        )
        leader_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='leader_world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'leader/base_link'],
            output='screen',
        )

        bi_teleop_node = Node(
            package='soa_teleop',
            executable='bi_teleop_node',
            name='bi_teleop_node',
            output='screen',
        )

        all_actions += [leader_rsp, leader_cm] + leader_spawners
        all_actions += [follower_static_tf, leader_static_tf, bi_teleop_node]

    # --- Display (RViz) ---
    if display == 'true':
        rviz_config = os.path.join(bringup_share, 'rviz', 'bi_soa.rviz')
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )
        all_actions.append(rviz)

    # --- Cameras ---
    if cameras == 'true' and use_sim == 'false':
        cameras_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'include', 'cameras.launch.py')
            ),
            launch_arguments={'config_file': 'bi_soa_cameras.yaml'}.items(),
        )
        all_actions.append(cameras_launch)

    # --- Simulation note ---
    if use_sim == 'true':
        all_actions.append(LogInfo(msg=(
            '[bi_soa_bringup] use_sim:=true — ensure Gazebo is running via the '
            'soa_sim package before controllers are spawned.'
        )))

    return all_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'leader',
            default_value='false',
            description='Launch bimanual leader arms in addition to followers. '
                        'Leader and follower arm pairs overlap in RViz for teleoperation monitoring.',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use Gazebo simulation for the follower arms. '
                        'When leader:=true, leaders still use real hardware.',
        ),
        DeclareLaunchArgument(
            'display',
            default_value='false',
            description='Launch RViz for visualization.',
        ),
        DeclareLaunchArgument(
            'cameras',
            default_value='true',
            description='Launch camera nodes (usb_cam / realsense2_camera). '
                        'Cameras are always on the follower arms only.',
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='forward',
            description='Controller type: "jtc" (JointTrajectory) or '
                        '"forward" (ForwardCommand).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
