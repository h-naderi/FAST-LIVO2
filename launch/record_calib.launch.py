#!/usr/bin/python3
"""
One-command static-scene recording for FAST-Calib LiDAR-camera extrinsic calibration.

Sibling of record_go2_lean.launch.py, but stripped to exactly what FAST-Calib reads
and raised to a resolution ArUco detection can actually work with.

DIFFERENCES FROM record_go2_lean.launch.py, AND WHY
===================================================
1. NO IMU STACK. FAST-Calib never reads an IMU -- it consumes one point cloud topic
   (src/FAST-Calib-ROS2/src/data_preprocess.hpp reads only `lidar_topic` from the
   bag) and one image file. Dropping lowstate_to_imu removes ~88% of a core that
   would otherwise compete with the recorder for UDP fragments.

2. COLOUR AT 1280x720, not 424x240. At 424x240 an 80 mm ArUco marker at 1.0 m spans
   ~25 px = ~3 px per marker module, which will not detect reliably. At 1280x720 it
   spans ~74 px = ~9 px per module, and the marker perimeter (~296 px) sits 7.7x
   above OpenCV's default minMarkerPerimeterRate gate of 0.03 * 1280 = 38 px.

   This resolution is for CALIBRATION ONLY. Do not change the mapping launches:
   the extrinsic FAST-Calib produces is a rigid transform and is resolution-
   independent, and 424x240 is a proportional 16:9 downscale of 1280x720
   (cx/width = 0.5027 in both), so the optical frame is identical.

3. NO MOTION PLAN. The scene must be perfectly static -- see below.

PHYSICAL SETUP -- THIS IS THE PART THAT DECIDES WHETHER IT WORKS
===============================================================
  * Board 1.00 m from the LiDAR. Measured from a real bag, the XT16 has 16 rings at
    2.007 deg pitch, so ring spacing is 35 mm at 1.0 m and a D160 mm hole is crossed
    by only ~4.6 rings. At the FAST-Calib default 1.5-3.0 m it is 2-3 rings, which
    is not enough to fit a circle. Closer is better for rings but the hole pattern
    spans 380 mm vertically against a 536 mm vertical FOV at 1.0 m, so 1.0 m is the
    balance point. 0.85 m is the fallback (5.4 rings, 456 mm FOV).
  * Board centre at LiDAR height, face square to the robot.
  * >= 2 m of open space BEHIND the board. The holes must read as genuine no-return;
    a wall behind fills them with points and hole extraction fails.
  * Robot and board COMPLETELY STILL for the whole recording. All messages in the
    bag are accumulated into one cloud, so any motion smears the board plane.
  * No other DICT_6X6_250 marker anywhere in view -- qr_detect.hpp:201 rejects the
    frame when MORE than 4 markers are detected, not just fewer than 4.

USAGE
=====
  source ~/fastlivo_ws/setup.sh
  ros2 launch fast_livo record_calib.launch.py bag_prefix:=go2_scene1

  # optional
  ros2 launch fast_livo record_calib.launch.py \
      bag_prefix:=go2_scene2 out_dir:=~/fastlivo_ws/src/FAST-Calib-ROS2/calib_data

Check net.core.rmem_max first -- it resets to 212992 on every reboot, and at the
stock value the recorder drops ~39% of scans:
  cat /proc/sys/net/core/rmem_max
  sudo sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864

Recording starts ~8 s in. ~20 s of stillness is plenty; stop with ONE Ctrl+C.

Three scenes are needed: neither ROS2 fork ported upstream's multi_calib, so each
run is single-scene. Record the board facing forward, angled right, and angled left,
then compare the three results.
"""

import os
import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess, OpaqueFunction, LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# Exactly what FAST-Calib needs, and nothing else. camera_info is included so the
# intrinsics can be read back from the bag rather than trusted from another run.
RECORD_TOPICS = [
    '/lidar_points',              # Hesai XT16 -> FAST-Calib `lidar_topic`
    '/camera/color/image_raw',    # one frame is extracted to PNG for `image_path`
    '/camera/color/camera_info',  # fx/fy/cx/cy at the recorded resolution
]

# Colour profile for the calibration recording only. See docstring point 2.
RGB_PROFILE = '1280x720x15'

_C_BANNER = '\033[1;92m'   # bright green
_C_WARN   = '\033[1;93m'   # bright yellow
_C_OFF    = '\033[0m'


def _launch_recorder(context, *args, **kwargs):
    """Build the record command at launch time so the name carries a timestamp."""
    out_dir = os.path.expanduser(context.launch_configurations['out_dir'])
    prefix = context.launch_configurations['bag_prefix']
    os.makedirs(out_dir, exist_ok=True)

    stamp = datetime.datetime.now().strftime('%m%d_%H%M%S')
    bag_path = os.path.join(out_dir, f'{prefix}_{stamp}')

    # `ros2 bag record` is a single in-process Python recorder, not a wrapper that
    # spawns a child, so launch's SIGINT reaches it and the bag closes cleanly. The
    # timeouts are still raised from launch's 5 s + 5 s defaults so a SIGKILL can
    # never land mid-close and leave the bag without its metadata.yaml.
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path] + RECORD_TOPICS,
        name='bag_recorder',
        output='screen',
        sigterm_timeout='120',
        sigkill_timeout='60',
    )

    return [
        LogInfo(msg=(
            f'\n{_C_BANNER}=== CALIBRATION RECORDING -> {bag_path} ==={_C_OFF}\n'
            f'{_C_BANNER}=== colour {RGB_PROFILE}, no IMU stack ==={_C_OFF}\n'
            f'{_C_WARN}>>> HOLD EVERYTHING STILL -- robot and board -- for the whole run.{_C_OFF}\n'
            f'    Every message in the bag is accumulated into one cloud, so any\n'
            f'    motion smears the board plane and the hole fit fails.\n'
            f'{_C_WARN}>>> Board 1.00 m out, centred at LiDAR height, open space behind.{_C_OFF}\n'
            f'{_C_WARN}>>> ~20 s is plenty. Stop with ONE Ctrl+C.{_C_OFF}\n'
        )),
        recorder,
    ]


def generate_launch_description():

    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix', default_value='go2_calib',
        description='Bag name prefix; a _MMDD_HHMMSS timestamp is appended')

    out_dir_arg = DeclareLaunchArgument(
        'out_dir', default_value='~/fastlivo_ws/src/FAST-Calib-ROS2/calib_data',
        description='Directory the bag is written into')

    record_delay_arg = DeclareLaunchArgument(
        'record_delay', default_value='8.0',
        description='Seconds to wait for drivers before recording starts')

    rgb_profile_arg = DeclareLaunchArgument(
        'rgb_profile', default_value=RGB_PROFILE,
        description='RealSense colour profile WxHxFPS for the calibration image')

    # -- 1. Hesai LiDAR driver (t=0s) -> /lidar_points --
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hesai_lidar_driver'), 'launch', 'start.py'])
        )
    )

    # -- 2. RealSense D435i (t=3s) -> /camera/color/image_raw at 1280x720 --
    realsense_delayed = TimerAction(period=3.0, actions=[
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
                'rgb_camera.profile': LaunchConfiguration('rgb_profile'),
            }.items()
        )
    ])

    # -- 3. Recorder: last, so no topic is missing when the bag opens --
    recorder_delayed = TimerAction(
        period=LaunchConfiguration('record_delay'),
        actions=[OpaqueFunction(function=_launch_recorder)]
    )

    return LaunchDescription([
        bag_prefix_arg,
        out_dir_arg,
        record_delay_arg,
        rgb_profile_arg,
        lidar,
        realsense_delayed,
        recorder_delayed,
    ])
