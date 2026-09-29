#!/usr/bin/python3
"""
One-command recording for the Go2 + Hesai XT16 + D435i rig -- lean IMU stack.

Lean counterpart to record_go2_debug.launch.py. Same drivers, same motion cues,
same bag layout; the difference is the IMU stack and what that buys you.

WHY LEAN, FOR A RECORDING
=========================
record_go2_debug.launch.py includes go2_driver/launch/imu.launch.py, which starts
three nodes. Two of them are pure overhead at record time:

  imu_calib_republisher  ~81% CPU -> /imu/data_corrected, /imu/data_gyro_only
  imu_filter_madgwick    ~13% CPU -> /imu/filtered

Together ~94% of a core. On this Jetson that competes directly with the recorder,
and a dropped UDP fragment discards a whole 0.97 MB XT16 scan -- so the cost lands
as missing scans in the bag, which is the one defect you cannot fix afterwards.

Nothing is actually lost by dropping them. Both derive from /imu/data, which
derives from /lowstate, and BOTH of those are recorded below. Any IMU variant can
be regenerated offline by replaying /lowstate through the same nodes. What cannot
be regenerated is a scan that was never written.

Also note FAST-LIVO2 subscribes to /imu/data (go2_xt16.yaml `common.imu_topic`),
not /imu/filtered -- so the lean set is exactly what the mapper consumes, plus
/lowstate for offline IMU work.

Use record_go2_debug.launch.py instead when you specifically want the calibrated
IMU topics captured live -- e.g. comparing republisher output against raw gyro
with matched timestamps.

USAGE
=====
  source ~/fastlivo_ws/setup.sh
  ros2 launch fast_livo record_go2_lean.launch.py bag_prefix:=final_test

  # optional
  ros2 launch fast_livo record_go2_lean.launch.py \
      bag_prefix:=final_test record_delay:=15.0

Check net.core.rmem_max is 67108864 first -- it resets to 212992 on every reboot,
and at the stock value the recorder drops ~39% of scans:
  cat /proc/sys/net/core/rmem_max
  sudo sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864

Recording starts ~10 s in, once all drivers are publishing, and the console prints
the bag path when it does. There is no timed motion plan -- explore however you
like. The one constraint: HOLD STILL for ~5 s after the banner appears, because
IMU init averages gravity and gyro bias over the first 30 LiDAR frames (~3 s at
10 Hz). Then stop with a single Ctrl+C and wait for rosbag2 to close the file.

If you do want a scripted routine with timestamped cues written to
<bag>_cues.txt, use record_go2_debug.launch.py instead.

Rate is roughly 13 MB/s (~780 MB/min). Replay with mapping_go2_bag.launch.py.
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Topics captured. Everything the mapper consumes, plus /lowstate so the whole
# IMU chain can be rebuilt offline. Keep /lowstate last so it is easy to drop if
# the bag ever needs to be replayable without unitree_go.
RECORD_TOPICS = [
    # --- LIO ---
    '/lidar_points',            # Hesai XT16, per-point timestamps for undistortion
    '/imu/data',                # what FAST-LIVO2 subscribes to (go2_xt16.yaml)
    # --- VIO ---
    '/camera/color/image_raw',
    '/camera/color/camera_info',
    '/tf_static',
    # --- offline IMU work: every other /imu/* topic derives from this ---
    '/lowstate',                # carries the MCU tick field
]

# ANSI colours, picked so the banner is impossible to miss scrolling past driver output.
_C_BANNER = '\033[1;92m'   # bright green
_C_WARN   = '\033[1;93m'   # bright yellow
_C_OFF    = '\033[0m'

# No timed motion plan here -- this launch is for free exploration, drive the robot
# however you like. The ONE constraint is the static hold at the start, below.
#
# ImuProcess::IMU_init (IMU_Processing.cpp:107) estimates gravity direction and gyro
# bias as a running mean of raw acc/gyro over `imu_int_frame` LiDAR frames --
# go2_xt16.yaml sets 30, which at 10 Hz is ~3 s. Move during that window and the
# mean acc picks up motion acceleration instead of pure gravity, and the mean gyro
# picks up real rotation instead of bias, so both come out wrong. Note this is
# consumed when the bag is REPLAYED, so what matters is that the bag *begins*
# static; 5 s of stillness after the banner appears is plenty.


def _launch_recorder(context, *args, **kwargs):
    """Build the record command at launch time so the name carries a timestamp."""
    out_dir = os.path.expanduser(context.launch_configurations['out_dir'])
    prefix = context.launch_configurations['bag_prefix']
    os.makedirs(out_dir, exist_ok=True)

    stamp = datetime.datetime.now().strftime('%m%d_%H%M%S')
    bag_path = os.path.join(out_dir, f'{prefix}_{stamp}')

    # `ros2 bag record` is a single Python process with rosbag2 in-process -- it is
    # NOT a wrapper that spawns a child, so launch's SIGINT reaches it and the bag
    # closes cleanly (verified: metadata.yaml written, exit in ~2 s). The timeouts
    # below are still raised from launch's 5 s + 5 s defaults, because flushing and
    # closing a multi-GB sqlite file takes longer than 5 s and SIGKILL mid-close
    # would leave the bag without its metadata.yaml.
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path] + RECORD_TOPICS,
        name='bag_recorder',
        output='screen',
        sigterm_timeout='120',
        sigkill_timeout='60',
    )

    return [
        LogInfo(msg=(
            f'\n{_C_BANNER}=== RECORDING -> {bag_path} ==={_C_OFF}\n'
            f'{_C_BANNER}=== lean IMU stack: /imu/data + /lowstate only ==={_C_OFF}\n'
            f'{_C_WARN}>>> HOLD STILL ~5 s, then explore freely.{_C_OFF}\n'
            f'    IMU init averages gravity and gyro bias over the first 30 LiDAR\n'
            f'    frames (~3 s at 10 Hz); moving through that window corrupts both.\n'
            f'{_C_WARN}>>> Stop with ONE Ctrl+C, then wait for the bag to close.{_C_OFF}\n'
        )),
        recorder,
    ]


def generate_launch_description():

    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix', default_value='go2_lean',
        description='Bag name prefix; a _MMDD_HHMMSS timestamp is appended')

    out_dir_arg = DeclareLaunchArgument(
        'out_dir', default_value='~/fastlivo_ws/rosbags_go2',
        description='Directory the bag is written into')

    record_delay_arg = DeclareLaunchArgument(
        'record_delay', default_value='10.0',
        description='Seconds to wait for drivers before recording starts')

    # ── 1. IMU (t=0s): raw /lowstate -> /imu/data. ONLY this node -- no
    #       imu_calib_republisher, no imu_filter_madgwick. See module docstring.
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
    # Same stream config as mapping_go2_lean.launch.py so the bag replays identically.
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

    # ── 4. Recorder: last, so no topic is missing when the bag opens ──
    recorder_delayed = TimerAction(
        period=LaunchConfiguration('record_delay'),
        actions=[OpaqueFunction(function=_launch_recorder)]
    )

    return LaunchDescription([
        bag_prefix_arg,
        out_dir_arg,
        record_delay_arg,
        imu_node,
        lidar_delayed,
        realsense_delayed,
        recorder_delayed,
    ])
