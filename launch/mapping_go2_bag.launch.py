#!/usr/bin/python3
"""Bag-replay variant of mapping_go2.launch.py.

Same mapping + camera-blackboard + rviz as the live launch, but WITHOUT
starting any sensor drivers (IMU / Hesai / RealSense). Subscribe to the
same topics — they come from `ros2 bag play` instead.

Usage:
  # Terminal A
  ros2 launch fast_livo mapping_go2_bag.launch.py use_rviz:=True
  # Terminal B (foxy LD_LIBRARY_PATH workaround from CLAUDE.md)
  source /opt/ros/foxy/setup.bash && source ~/fastlivo_ws/install/setup.bash && \
    LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH \
    ros2 bag play ~/fastlivo_ws/rosbags_go2/go2_livo_test --rate 1.0
"""

import os
import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

_REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_LOG_DIR = os.path.join(_REPO_DIR, 'logs')


def _launch_mapping(context, *args, **kwargs):
    os.makedirs(_LOG_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(_LOG_DIR, f'fastlivo_bag_{timestamp}.log')

    params_file = context.launch_configurations['main_params_file']

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

    # Camera-intrinsics blackboard (vikit reads camera params from here)
    blackboard_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        parameters=[LaunchConfiguration('camera_params_file')],
        output='screen'
    )

    # Give the blackboard ~1s to come up before the mapper queries it
    mapping_delayed = TimerAction(
        period=2.0,
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
        blackboard_node,
        mapping_delayed,
        rviz_node,
    ])
