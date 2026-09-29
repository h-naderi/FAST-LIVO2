#!/usr/bin/python3
"""
One-command debug recording for the Go2 + Hesai XT16 + D435i rig.

Brings up the same three driver stacks as `mapping_go2.launch.py` (IMU chain,
Hesai LiDAR, RealSense colour) but runs `ros2 bag record` instead of the SLAM
node, capturing everything needed to reproduce and diagnose a run offline --
LIO, VIO, and the raw IMU chain used for timing/calibration checks.

Usage (source BOTH workspaces -- /lowstate needs unitree_go from MAX-Brain):
  source /opt/ros/foxy/setup.bash
  source ~/MAX-Brain-v1.0/install/setup.bash
  source ~/fastlivo_ws/install/setup.bash
  ros2 launch fast_livo record_go2_debug.launch.py

  # optional
  ros2 launch fast_livo record_go2_debug.launch.py bag_prefix:=yaw_test

Recording starts ~10 s in, once all drivers are publishing; the console prints
the bag path when it does. From then on it calls out each motion phase (stand /
walk / rotate) as it comes due -- just follow the prompts. Stop with a single
Ctrl+C and wait a few seconds for rosbag2 to close the file cleanly.

Each cue is also appended to `<bag>_cues.txt` with its epoch timestamp, which is
the same clock the message headers use -- so a phase can be located in the bag
exactly rather than guessed at. Edit MOTION_PLAN below to change the routine.

Rate is roughly 15 MB/s, so a 90 s run is ~1.4 GB. Replay it with
`mapping_go2_bag.launch.py`.
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

# Topics captured. Grouped by what each is for -- keep /lowstate last so it is
# easy to drop if the bag ever needs to be replayable without unitree_go.
RECORD_TOPICS = [
    # --- LIO ---
    '/lidar_points',            # Hesai XT16, per-point timestamps for undistortion
    '/imu/filtered',            # Madgwick output; kept for comparison only --
                                # FAST-LIVO2 subscribes to /imu/data, see below
    # --- VIO ---
    '/camera/color/image_raw',
    '/camera/color/camera_info',
    '/tf_static',
    # --- IMU chain diagnostics (stamp propagation, gyro calibration) ---
    '/imu/data',                # pre-calibration, arrival-stamped
    '/imu/data_corrected',      # post bias + cross-talk correction
    '/imu/data_gyro_only',
    '/lowstate',                # carries the MCU tick field
]

# Motion routine, called out on the console once recording is live.
# (duration_s, label, what to actually do)
MOTION_PLAN = [
    (10.0, 'STAND STILL',         'do not touch the robot -- IMU init needs 30 LiDAR frames'),
    (10.0, 'WALK STRAIGHT',       'forward ~5 m, steady pace, no turning'),
    ( 5.0, 'STAND STILL',         'let the state settle'),
    (12.0, 'ROTATE 360 IN PLACE', 'slow yaw, ~30 deg/s, stay on the spot'),
    (10.0, 'DONE - STAND STILL',  'hold, then press Ctrl+C ONCE to close the bag'),
]

# ANSI colours, picked so a cue is impossible to miss scrolling past driver output.
_C_BANNER = '\033[1;92m'   # bright green
_C_CUE    = '\033[1;93m'   # bright yellow
_C_OFF    = '\033[0m'


def _make_cue(cue_file, label, detail, elapsed, duration):
    """Build the OpaqueFunction that fires one motion cue at its scheduled time."""
    def _cue(context, *args, **kwargs):
        now = datetime.datetime.now()
        try:
            with open(cue_file, 'a') as f:
                f.write(f'{now.timestamp():.3f}  t+{elapsed:05.1f}s  {label}\n')
        except OSError:
            pass  # a cue log failure must never take the recording down
        return [LogInfo(msg=(
            f'\n{_C_CUE}>>> [t+{elapsed:.0f}s] {label} '
            f'({duration:.0f}s){_C_OFF}  {detail}\n'
        ))]
    return _cue


def _launch_recorder(context, *args, **kwargs):
    """Build the record command at launch time so the name carries a timestamp."""
    out_dir = os.path.expanduser(context.launch_configurations['out_dir'])
    prefix = context.launch_configurations['bag_prefix']
    os.makedirs(out_dir, exist_ok=True)

    stamp = datetime.datetime.now().strftime('%m%d_%H%M%S')
    bag_path = os.path.join(out_dir, f'{prefix}_{stamp}')
    cue_file = f'{bag_path}_cues.txt'

    # Timeouts raised from launch's 5 s + 5 s defaults: flushing and closing a
    # multi-GB sqlite file takes longer than 5 s, and a SIGKILL mid-close leaves
    # the bag without its metadata.yaml. (`ros2 bag record` itself is fine -- it is
    # a single in-process Python recorder, not a wrapper that spawns a child, so
    # SIGINT does reach it. Verified: clean close, metadata.yaml written, ~2 s.)
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path] + RECORD_TOPICS,
        name='bag_recorder',
        output='screen',
        sigterm_timeout='120',
        sigkill_timeout='60',
    )

    total = sum(d for d, _, _ in MOTION_PLAN)
    actions = [
        LogInfo(msg=(
            f'\n{_C_BANNER}=== RECORDING -> {bag_path} ==={_C_OFF}\n'
            f'{_C_BANNER}=== motion plan: {total:.0f}s, follow the prompts below ==={_C_OFF}\n'
        )),
        recorder,
    ]

    # Timers are created here, so their clocks start when recording does.
    elapsed = 0.0
    for duration, label, detail in MOTION_PLAN:
        actions.append(TimerAction(
            period=elapsed,
            actions=[OpaqueFunction(
                function=_make_cue(cue_file, label, detail, elapsed, duration))]
        ))
        elapsed += duration

    return actions


def generate_launch_description():

    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix', default_value='go2_debug',
        description='Bag name prefix; a _MMDD_HHMMSS timestamp is appended')

    out_dir_arg = DeclareLaunchArgument(
        'out_dir', default_value='~/fastlivo_ws/rosbags_go2',
        description='Directory the bag is written into')

    record_delay_arg = DeclareLaunchArgument(
        'record_delay', default_value='10.0',
        description='Seconds to wait for drivers before recording starts')

    # ── 1. IMU pipeline (t=0s): lowstate -> imu_calib_republisher -> Madgwick ──
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('go2_driver'), 'launch', 'imu.launch.py'])
        )
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
    # Same stream config as mapping_go2.launch.py so the bag replays identically.
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
        imu_launch,
        lidar_delayed,
        realsense_delayed,
        recorder_delayed,
    ])
