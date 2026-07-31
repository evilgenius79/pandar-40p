#!/usr/bin/env python3
"""One-command rig startup: lidar driver + sensor bridge (+ optional record).

The Hesai driver launch spawns its own preconfigured RViz window (already
pointed at /lidar_points with the right frame), so this file no longer starts
a second one — that was the extra-window fix.

Usage:
    ros2 launch ~/rig.launch.py
    ros2 launch ~/rig.launch.py record:=true

Edit the three CONFIG paths once to match your machine.
"""
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ---------------- CONFIG: adjust these three paths once ----------------
HESAI_LAUNCH = os.path.expanduser(
    "~/ros2_ws/src/HesaiLidar_ROS_2.0/launch/start.py")
BRIDGE_SCRIPT = os.path.expanduser(
    "~/Desktop/imu_bridge_node/imu_bridge_node.py")
BAG_DIR = os.path.expanduser("~/bags")
# ----------------------------------------------------------------------

BRIDGE_PORT = "/dev/ttyACM0"
RECORD_TOPICS = ["/lidar_points", "/imu/data_raw", "/gps/fix", "/gps/pps"]


def generate_launch_description():
    record_arg = DeclareLaunchArgument("record", default_value="false")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(HESAI_LAUNCH))

    bridge = ExecuteProcess(
        cmd=["python3", BRIDGE_SCRIPT, "--port", BRIDGE_PORT],
        name="sensor_bridge", output="screen")

    bag_path = os.path.join(BAG_DIR,
                            datetime.now().strftime("run_%Y%m%d_%H%M%S"))
    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_path] + RECORD_TOPICS,
        name="bag_record", output="screen",
        condition=IfCondition(LaunchConfiguration("record")))

    return LaunchDescription([record_arg, lidar, bridge, record])
