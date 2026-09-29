#!/usr/bin/python3
"""Lean variant of mapping_go2.launch.py -- raw IMU only.

Identical to mapping_go2.launch.py except for the IMU stack. Instead of
including go2_driver/launch/imu.launch.py (which starts three nodes), this
starts ONLY lowstate_to_imu, publishing /imu/data.

Why the other two are dropped:

  imu_calib_republisher  ~81% CPU. Applies ~/imu_calib_data.yaml, whose
      ang_z2x_proj / ang_z2y_proj inject ~45 deg of roll and ~70 deg of pitch
      per 360 deg of yaw -- measured against the raw gyro, which integrates to
      -7.4 / +7.8 deg over the same turn. Its acc_bias_x/y (-1.32 / +1.37) are
      gravity projections from a tilted calibration pose, not biases; they tilt
      the apparent gravity vector by 9.7 deg (raw reads 1.3 deg from level).
      This is what made the map diverge on every rotation.

  imu_filter_madgwick    ~13% CPU. Only fills header.orientation, which
      FAST-LIVO2 never reads -- it uses angular_velocity and linear_acceleration
      and estimates its own attitude.

Together they burned ~94% of a core producing topics nothing subscribes to,
starving lowstate_to_imu (~88% CPU) and backlogging the mapper's IMU queue by
0.5-2.3 s. Set `common.imu_topic: "/imu/data"` in go2_xt16.yaml to match.

Nothing in MAX-Brain-v1.0 is modified; this simply does not launch those nodes.

Usage:
  ros2 launch fast_livo mapping_go2_lean.launch.py use_rviz:=True
"""

import os
import glob
import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess, OpaqueFunction, LogInfo, RegisterEventHandler
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_LOG_DIR = os.path.join(_REPO_DIR, 'logs')


def _launch_mapping(context, *args, **kwargs):
    """Spawned via OpaqueFunction so the log path carries a launch-time stamp."""
    os.makedirs(_LOG_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(_LOG_DIR, f'fastlivo_lean_{timestamp}.log')

    params_file = context.launch_configurations['main_params_file']

    # savePCD() runs only after the `while (rclcpp::ok())` loop in LIVMapper::run(),
    # so the map is written only if SIGINT actually reaches the C++ process. Two
    # wrapper layers used to prevent that, and both are removed here:
    #
    #   1. `bash -c 'cmd 2>&1 | tee log'` -- bash sits in wait() and does not
    #      forward SIGINT to the pipeline. Measured: the mapper kept running and
    #      launch escalated to SIGKILL, which cannot be caught. And on a terminal
    #      Ctrl+C (which signals the whole process group) tee died at the same
    #      instant, so the mapper took a SIGPIPE part-way through the save -- that
    #      is how Log/pcd/ ended up with all_raw_points.pcd but no
    #      all_downsampled_points.pcd. Logging now goes through launch instead.
    #
    #   2. `ros2 run fast_livo fastlivo_mapping` -- also a Python wrapper that
    #      spawns the executable as a child without forwarding SIGINT. Measured:
    #      SIGINT to it left the mapper running; SIGINT to the binary itself saved
    #      both clouds and exited in ~2 s. Hence the absolute path below.
    mapper_bin = os.path.join(
        get_package_prefix('fast_livo'), 'lib', 'fast_livo', 'fastlivo_mapping')

    mapping = ExecuteProcess(
        cmd=[
            mapper_bin,
            '--ros-args', '--remap', '__node:=laserMapping',
            '--params-file', params_file,
        ],
        name='fastlivo_mapping',
        # 'full' = console + launch's main log + its own fastlivo_mapping-stdout.log
        output='full',
        # Defaults are 5 s + 5 s. Writing ~1.7M points takes longer than that on
        # this Jetson, and SIGKILL cannot be caught, so give the save real room.
        sigterm_timeout='90',
        sigkill_timeout='60',
    )

    # Keep the familiar logs/fastlivo_lean_<stamp>.log path working by symlinking it
    # to the file launch writes for this run. The real name carries a process index
    # (fastlivo_mapping-<N>-stdout.log) that is only known once the process exists,
    # so glob for it on the process-start event rather than guessing.
    def _link_log(event, context):
        try:
            from launch.logging import launch_config
            matches = glob.glob(os.path.join(
                launch_config.log_dir, 'fastlivo_mapping-*-stdout.log'))
            if not matches:
                return
            if os.path.islink(log_file) or os.path.exists(log_file):
                os.remove(log_file)
            os.symlink(max(matches, key=os.path.getmtime), log_file)
        except Exception as exc:   # never let logging cosmetics break the launch
            print(f'[mapping_go2_lean] could not link {log_file}: {exc}')

    return [
        mapping,
        RegisterEventHandler(OnProcessStart(target_action=mapping,
                                            on_start=_link_log)),
    ]


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

    # ── 1. IMU (t=0s): raw /lowstate -> /imu/data. No calib republisher, no madgwick. ──
    imu_node = Node(
        package='go2_driver',
        executable='lowstate_to_imu',
        name='lowstate_to_imu',
        output='screen'
    )

    # ── 2. Hesai LiDAR driver (t=3s) → /lidar_points ──
    lidar_delayed = TimerAction(period=3.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('hesai_lidar_driver'), 'launch', 'start.py'])
            )
        )
    ])

    # ── 3. RealSense D435i (t=5s) → /camera/color/image_raw ──
    realsense_delayed = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
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
    ])

    # ── 4. Camera intrinsics blackboard (t=8s) ──
    blackboard_delayed = TimerAction(period=8.0, actions=[
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[LaunchConfiguration('camera_params_file')],
            output='screen'
        )
    ])

    # ── 5. Main SLAM node (t=10s) ──
    mapping_delayed = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='\n\033[1;92m=== lean stack: /imu/data only '
                        '(no calib republisher, no madgwick) ===\033[0m\n'
                        '\033[1;93m    go2_xt16.yaml must have imu_topic: "/imu/data"\033[0m\n'),
            OpaqueFunction(function=_launch_mapping),
        ]
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
        imu_node,
        lidar_delayed,
        realsense_delayed,
        blackboard_delayed,
        mapping_delayed,
        rviz_node,
    ])
