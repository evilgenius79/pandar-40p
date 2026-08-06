#!/usr/bin/env python3
"""One-command rig startup: lidar driver + sensor bridge (+ optional record).

The Hesai driver launch spawns its own preconfigured RViz window (already
pointed at /lidar_points with the right frame), so this file no longer starts
a second one — that was the extra-window fix.

Usage:
    ros2 launch ~/pandar-40p/launch/rig.launch.py
    ros2 launch ~/pandar-40p/launch/rig.launch.py record:=true

Run it from the REPO path, not from a copy on the Desktop. Every node it
spawns now lives in the repo, so there is one file to edit and one file
under version control. An identical copy at ~/Desktop/rig_launch_v2.py is
what older notes referenced; it still works, but it is a copy and copies
drift.
"""
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ---------------- CONFIG: adjust these two paths once ----------------
HESAI_LAUNCH = os.path.expanduser(
    "~/ros2_ws/src/HesaiLidar_ROS_2.0/launch/start.py")
BAG_DIR = os.path.expanduser("~/bags")
# ---------------------------------------------------------------------

REPO = os.path.expanduser("~/pandar-40p")
# All three nodes come from the repo. BRIDGE_SCRIPT used to point at
# ~/Desktop/imu_bridge_node/ while the other two already came from here,
# which meant the file that actually ran was not the file under version
# control. They were byte-identical when this was changed (2026-08-06).
BRIDGE_SCRIPT = f"{REPO}/ros2/imu_bridge_node/imu_bridge_node.py"
LIDAR_TEMP = f"{REPO}/ros2/lidar_temp_node/lidar_temp_node.py"
RIG_STATUS = f"{REPO}/ros2/rig_status_node/rig_status_node.py"

# The IMU bridge's serial port. NOT a stable name: the LG290P GNSS module
# enumerates as a CH343 and also claims /dev/ttyACM0, so with both plugged
# in whichever appears first wins and the loser silently gets the wrong
# device. Install scripts/gnss/99-rig-serial.rules and this becomes
# /dev/imu. Until the XIAO's USB IDs are read from the board, that rule is
# deliberately incomplete -- see docs/rtk_gnss.md section 6.
BRIDGE_PORT = "/dev/ttyACM0"
RECORD_TOPICS = ["/lidar_points", "/imu/data_raw", "/gps/fix", "/gps/pps",
                 "/imu/temperature", "/lidar/temperature"]


def generate_launch_description():
    record_arg = DeclareLaunchArgument("record", default_value="false")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(HESAI_LAUNCH))

    bridge = ExecuteProcess(
        cmd=["python3", BRIDGE_SCRIPT, "--port", BRIDGE_PORT],
        name="sensor_bridge", output="screen")

    # Thermal context for drift analysis. The IMU's own temperature now rides
    # along in the bridge's stream; this adds the lidar's, polled from the
    # console API. Both are cheap and neither existed before 2026-08-01.
    lidar_temp = ExecuteProcess(
        cmd=["python3", LIDAR_TEMP],
        name="lidar_temp", output="log")

    # Status JSON on :8080. Read-only, BEST_EFFORT subscriptions only, so it
    # cannot add back-pressure to the pipeline it observes. Reachable over
    # Tailscale, so rig state is checkable from a phone mid-run.
    rig_status = ExecuteProcess(
        cmd=["python3", RIG_STATUS],
        name="rig_status", output="log")

    bag_path = os.path.join(BAG_DIR,
                            datetime.now().strftime("run_%Y%m%d_%H%M%S"))
    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_path] + RECORD_TOPICS,
        name="bag_record", output="screen",
        condition=IfCondition(LaunchConfiguration("record")))

    return LaunchDescription(
        [record_arg, lidar, bridge, lidar_temp, rig_status, record])
