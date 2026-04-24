#!/usr/bin/python3

import os
import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Resolve log dir to logs/ inside the FAST-LIVO2 repo (works with --symlink-install)
_REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_LOG_DIR = os.path.join(_REPO_DIR, 'logs')


def _launch_mapping(context, *args, **kwargs):
    """Spawned via OpaqueFunction so we can compute a timestamped log path at launch time."""
    os.makedirs(_LOG_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(_LOG_DIR, f'fastlivo_{timestamp}.log')

    params_file = context.launch_configurations['main_params_file']

    # tee: output goes to screen AND is saved to log_file simultaneously
    mapping = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'ros2 run fast_livo fastlivo_mapping '
            f'--ros-args --remap __node:=laserMapping --params-file {params_file} '
            f'2>&1 | tee {log_file}'
        ],
        output='screen'
    )
    return [mapping]


def generate_launch_description():

    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    main_config   = os.path.join(config_file_dir, "go2_xt16.yaml")
    camera_config = os.path.join(config_file_dir, "camera_go2_d435i.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Launch Rviz2")

    main_config_arg = DeclareLaunchArgument(
        'main_params_file', default_value=main_config,
        description='Main FAST-LIVO2 parameter file')

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file', default_value=camera_config,
        description='Camera intrinsics parameter file (vikit)')

    # ── 1. IMU pipeline (t=0s): lowstate → imu_calib_republisher → Madgwick → /imu/filtered ──
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('go2_driver'), 'launch', 'imu.launch.py'])
        )
    )

    # ── 2. Hesai LiDAR driver (t=3s) → /lidar_points ──
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hesai_lidar_driver'), 'launch', 'start.py'])
        )
    )
    lidar_delayed = TimerAction(period=3.0, actions=[lidar_launch])

    # ── 3. RealSense D435i (t=5s) → /camera/color/image_raw ──
    # Depth/IMU streams disabled: FAST-LIVO2 uses LiDAR for depth and GO2 IMU for inertial.
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'enable_color':       'true',
            'enable_depth':       'false',
            'enable_sync':        'false',
            'enable_gyro':        'false',
            'enable_accel':       'false',
            'rgb_camera.profile': '424x240x15',
        }.items()
    )
    realsense_delayed = TimerAction(period=5.0, actions=[realsense_launch])

    # ── 4. Camera intrinsics blackboard (t=8s): vikit reads camera params from here ──
    blackboard_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        parameters=[LaunchConfiguration('camera_params_file')],
        output='screen'
    )
    blackboard_delayed = TimerAction(period=8.0, actions=[blackboard_node])

    # ── 5. Main SLAM node (t=10s): all sensors publishing, blackboard ready ──
    # Output goes to screen AND ~/fastlivo_ws/src/FAST-LIVO2/logs/fastlivo_<timestamp>.log
    mapping_delayed = TimerAction(
        period=10.0,
        actions=[OpaqueFunction(function=_launch_mapping)]
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    return LaunchDescription([
        use_rviz_arg,
        main_config_arg,
        camera_config_arg,
        imu_launch,
        lidar_delayed,
        realsense_delayed,
        blackboard_delayed,
        mapping_delayed,
        rviz_node,
    ])
