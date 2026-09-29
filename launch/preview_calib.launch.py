#!/usr/bin/python3
"""
Live preview of the FAST-Calib sensor setup -- drivers only, NO recorder.

Run this before record_calib.launch.py to confirm the board is placed correctly and
both streams look right. Same driver config as record_calib.launch.py, so what you
see here is exactly what the bag will contain.

WHAT IT STARTS
==============
  t=0s  Hesai XT16 driver          -> /lidar_points
  t=0s  rosbridge websocket :9090  -> for Foxglove   (bridge:=false to skip)
  t=3s  RealSense D435i @1280x720  -> /camera/color/image_raw
  t=12s one-shot sanity check, prints stream geometry and point extents, then exits

The sanity check is the point of this launch file. It answers the three questions
that decide whether a recording is worth making, none of which are obvious in a
Foxglove 3D view:

  1. Did the colour stream ACTUALLY come up at 1280x720? RealSense silently falls
     back to another profile if the requested one is unsupported, and the intrinsics
     in qr_params_go2.yaml are only valid at 1280x720.
  2. Is the board at the right distance? At 1.00 m the XT16's 2.007 deg ring pitch
     puts ~4.6 rings across a D160 mm hole. At 1.5 m it is 3.0 and the circle fit
     fails.
  3. Where is the board in LiDAR coordinates? /lidar_points carries a baked-in
     +90 deg yaw from the driver (hesai_lidar_driver/config/config.yaml:51,57), so
     which axis points forward is NOT assumable -- and FAST-Calib's x/y/z_min/max
     distance filter is expressed in exactly these coordinates. The printed extents
     are what those six parameters should be set from.

USAGE
=====
  source ~/fastlivo_ws/setup.sh
  ros2 launch fast_livo preview_calib.launch.py

  # already running rosbridge elsewhere
  ros2 launch fast_livo preview_calib.launch.py bridge:=false

  # re-run the check without restarting the drivers: just relaunch, or raise
  ros2 launch fast_livo preview_calib.launch.py check_delay:=25.0

  # if the board is not at ~1.00 m, tell the check where to look for it
  BOARD_DIST=1.20 ros2 launch fast_livo preview_calib.launch.py

IN FOXGLOVE
===========
Connect to ws://<jetson-ip>:9090 (Rosbridge connection type).
  * Image panel  -> /camera/color/image_raw
      All four ArUco markers fully inside the frame, in focus, evenly lit. No
      glare on the marker faces -- specular highlight kills corner detection.
  * 3D panel     -> /lidar_points, display frame `hesai_lidar`
      (confirmed: that is the frame_id the driver stamps)
      The four holes must appear as clean voids. If points fall inside a hole you
      are seeing the wall behind the board -- move the board further from it.
      Colour by `ring` to see how many rings cross each hole: you want 4-5.

Ctrl+C when satisfied, then record with:
  ros2 launch fast_livo record_calib.launch.py bag_prefix:=go2_scene1
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    ExecuteProcess, LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
)
from launch_ros.substitutions import FindPackageShare

# Must match record_calib.launch.py, or the preview is not what gets recorded.
RGB_PROFILE = '1280x720x15'

_C_BANNER = '\033[1;92m'
_C_WARN   = '\033[1;93m'
_C_OFF    = '\033[0m'

# One-shot check. Grabs one CameraInfo and one PointCloud2, prints what matters,
# exits. Runs as its own process so it cannot wedge the drivers.
#
# NOTE the BEST_EFFORT QoS on /lidar_points: the Hesai driver publishes BEST_EFFORT,
# and a RELIABLE subscription silently receives nothing (the same trap documented for
# FAST-LIVO2's own subscriptions in the workspace CLAUDE.md).
CHECK_SCRIPT = r'''
import math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

G, Y, R, O = "\033[1;92m", "\033[1;93m", "\033[1;91m", "\033[0m"

def _ring_count(msg, expect):
    """Distinct `ring` indices with points within +-0.25 m of `expect`."""
    if not any(f.name == "ring" for f in msg.fields):
        return None
    seen = set()
    for x, y, z, ring in pc2.read_points(
            msg, field_names=("x", "y", "z", "ring"), skip_nans=True):
        if abs(math.sqrt(x * x + y * y + z * z) - expect) < 0.25:
            seen.add(int(ring))
    return len(seen)


class Check(Node):
    def __init__(self):
        super().__init__("calib_preview_check")
        self.cam = None
        self.cloud = None
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self._cam, 10)
        self.create_subscription(PointCloud2, "/lidar_points", self._cloud, be)

    def _cam(self, m):
        if self.cam is None:
            self.cam = m

    def _cloud(self, m):
        if self.cloud is None:
            self.cloud = m

rclpy.init()
n = Check()
for _ in range(400):                      # up to ~20 s
    rclpy.spin_once(n, timeout_sec=0.05)
    if n.cam is not None and n.cloud is not None:
        break

print("")
print(G + "=" * 68 + O)
print(G + " FAST-Calib preview check" + O)
print(G + "=" * 68 + O)

# ---- camera ----
if n.cam is None:
    print(R + " CAMERA: no /camera/color/camera_info received." + O)
    print("         RealSense not up yet, or the profile failed to start.")
else:
    c = n.cam
    ok = (c.width, c.height) == (1280, 720)
    tag = (G + "OK" + O) if ok else (R + "MISMATCH" + O)
    print(" CAMERA  %dx%d  [%s]" % (c.width, c.height, tag))
    print("   fx=%.4f fy=%.4f cx=%.4f cy=%.4f" % (c.k[0], c.k[4], c.k[2], c.k[5]))
    print("   model=%s  D=%s" % (c.distortion_model,
                                 [round(d, 6) for d in c.d]))
    if not ok:
        print(R + "   -> qr_params_go2.yaml intrinsics are for 1280x720 ONLY." + O)
        print(R + "      Either fix the profile or copy the fx/fy/cx/cy above"
              " into the config." + O)
    elif abs(c.k[0] - 912.1945) > 1.0 or abs(c.k[2] - 643.4632) > 1.0:
        print(Y + "   -> differs from qr_params_go2.yaml; copy the values above."
              + O)
    else:
        print("   matches qr_params_go2.yaml")

# ---- lidar ----
if n.cloud is None:
    print(R + " LIDAR: no /lidar_points received." + O)
    print("        Driver not up, or a QoS/network problem.")
else:
    pts = [p for p in pc2.read_points(n.cloud, field_names=("x", "y", "z"),
                                      skip_nans=True)]
    print(" LIDAR   %d points/scan, frame_id='%s'"
          % (len(pts), n.cloud.header.frame_id))

    # Points plausibly on the board: 0.5-2.0 m out. These extents are exactly what
    # FAST-Calib's x/y/z_min/max are expressed in.
    sel = []
    for x, y, z in pts:
        r = math.sqrt(x * x + y * y + z * z)
        if 0.5 < r < 2.0:
            sel.append((x, y, z, r))
    if not sel:
        print(R + "   No points between 0.5 and 2.0 m -- nothing in front of the"
              " sensor." + O)
    else:
        xs = [p[0] for p in sel]; ys = [p[1] for p in sel]
        zs = [p[2] for p in sel]; rs = [p[3] for p in sel]
        print("   %d points in the 0.5-2.0 m shell" % len(sel))
        print("     x: %+.3f .. %+.3f     y: %+.3f .. %+.3f     z: %+.3f .. %+.3f"
              % (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
        print("     range: %.3f .. %.3f m   (nearest surface %.3f m)"
              % (min(rs), max(rs), min(rs)))

        # Which horizontal axis is "forward" -- the driver's +90 deg yaw makes this
        # unsafe to assume. The board is the dominant nearby surface, so forward is
        # the axis whose mean magnitude is largest.
        mx = sum(abs(v) for v in xs) / len(xs)
        my = sum(abs(v) for v in ys) / len(ys)
        fwd = "x" if mx > my else "y"
        print("     dominant horizontal axis (likely 'forward'): %s"
              "   mean|x|=%.3f mean|y|=%.3f" % (fwd, mx, my))

        # min(range) over the whole shell is NOT the board -- it is whatever is
        # nearest, usually the robot's own body or the floor. Isolate the board by
        # taking points in a window around the expected distance and counting the
        # distinct rings that actually land on it.
        import os
        expect = float(os.environ.get("BOARD_DIST", "1.00"))
        win = [p for p in sel if abs(p[3] - expect) < 0.25]
        print("")
        if len(win) < 200:
            print(R + "   Only %d points within +-0.25 m of the expected %.2f m "
                  "board distance." % (len(win), expect) + O)
            print(Y + "   Nothing board-like there. Set BOARD_DIST to the real "
                  "distance and re-run, or move the board." + O)
        else:
            wr = sorted(p[3] for p in win)
            d = wr[len(wr) // 2]                       # median, robust to stragglers
            rings_on_board = _ring_count(n.cloud, expect)
            pitch = 1000.0 * d * math.tan(math.radians(2.007))
            rings = 160.0 / pitch
            col = G if rings >= 4.0 else (Y if rings >= 3.0 else R)
            print("   BOARD at %.3f m (median of %d points in the window)"
                  % (d, len(win)))
            if rings_on_board is not None:
                print("     distinct LiDAR rings landing on it: %d of 16"
                      % rings_on_board)
            print("     ring pitch %.0f mm -> %s%.1f rings across a D160 mm "
                  "hole%s" % (pitch, col, rings, O))
            if rings < 4.0:
                print(Y + "   -> move the board CLOSER. Target ~1.00 m "
                      "(4.6 rings)." + O)
            zs_w = [p[2] for p in win]
            print("     board z extent %+.3f .. %+.3f m  (centre %+.3f)"
                  % (min(zs_w), max(zs_w), 0.5 * (min(zs_w) + max(zs_w))))
            if abs(0.5 * (min(zs_w) + max(zs_w))) > 0.05:
                print(Y + "   -> board centre is %+.0f mm off the LiDAR's "
                      "elevation zero. Centre it on the LIDAR, not the camera."
                      % (1000 * 0.5 * (min(zs_w) + max(zs_w))) + O)

print(G + "=" * 68 + O)
print(" Set qr_params_go2.yaml x/y/z_min/max from the extents above,")
print(" tight around the board -- a wall or floor left inside the box is the")
print(" most common cause of a bad plane fit.")
print(G + "=" * 68 + O)
print("")
sys.stdout.flush()
n.destroy_node()
rclpy.shutdown()
'''


def generate_launch_description():

    bridge_arg = DeclareLaunchArgument(
        'bridge', default_value='true',
        description='Start rosbridge_server on :9090 for Foxglove')

    rgb_profile_arg = DeclareLaunchArgument(
        'rgb_profile', default_value=RGB_PROFILE,
        description='RealSense colour profile WxHxFPS; must match record_calib')

    check_delay_arg = DeclareLaunchArgument(
        'check_delay', default_value='12.0',
        description='Seconds before the one-shot sanity check runs')

    # -- Hesai LiDAR (t=0) --
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('hesai_lidar_driver'),
                                  'launch', 'start.py'])
        )
    )

    # -- rosbridge for Foxglove (t=0). No foxglove_bridge on this box, so Foxglove
    #    connects with the Rosbridge connection type. --
    bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rosbridge_server'),
                                  'launch', 'rosbridge_websocket_launch.xml'])
        ),
        condition=IfCondition(LaunchConfiguration('bridge'))
    )

    # -- RealSense (t=3), identical config to record_calib.launch.py --
    realsense_delayed = TimerAction(period=3.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('realsense2_camera'),
                                      'launch', 'rs_launch.py'])
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

    # -- One-shot check (t=check_delay) --
    check_delayed = TimerAction(
        period=LaunchConfiguration('check_delay'),
        actions=[ExecuteProcess(
            cmd=['python3', '-c', CHECK_SCRIPT],
            name='calib_preview_check',
            output='screen',
        )]
    )

    banner = LogInfo(msg=(
        f'\n{_C_BANNER}=== FAST-Calib PREVIEW -- drivers only, nothing is being '
        f'recorded ==={_C_OFF}\n'
        f'    Foxglove: ws://<jetson-ip>:9090  (Rosbridge connection)\n'
        f'      Image panel -> /camera/color/image_raw   (all 4 markers in frame, '
        f'no glare)\n'
        f'      3D panel    -> /lidar_points, colour by `ring`  (holes = clean '
        f'voids, 4-5 rings each)\n'
        f'{_C_WARN}    A sanity check prints stream geometry and board extents in '
        f'~12 s.{_C_OFF}\n'
    ))

    return LaunchDescription([
        bridge_arg,
        rgb_profile_arg,
        check_delay_arg,
        banner,
        lidar,
        bridge,
        realsense_delayed,
        check_delayed,
    ])
