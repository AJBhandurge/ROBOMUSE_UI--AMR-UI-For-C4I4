#!/usr/bin/env python3
"""
mission_runner.py  —  AMR Waypoint Mission
Reads waypoints from /home/jd/ros2_ws/src/amr_ui/missions/mission.json
Navigates to each waypoint in order, loops forever until stopped.

mission.json format:
{
  "waypoints": [
    {"name": "Station A", "x": 1.0, "y": -0.5, "yaw": 0.0},
    {"name": "Station B", "x": 3.0, "y":  1.2, "yaw": 1.57}
  ]
}
"""

import sys
import json
import math
import time
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


MISSION_FILE = "/home/ayush/c_ws/src/amr_ui/missions/mission.json"


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(nav, x, y, yaw):
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main():
    # Load mission file
    try:
        with open(MISSION_FILE, "r") as f:
            mission = json.load(f)
    except Exception as e:
        print(f"[mission_runner] ERROR: Could not load mission file: {e}", flush=True)
        sys.exit(1)

    waypoints = mission.get("waypoints", [])
    if not waypoints:
        print("[mission_runner] ERROR: No waypoints in mission file.", flush=True)
        sys.exit(1)

    print(f"[mission_runner] Loaded {len(waypoints)} waypoints:", flush=True)
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. {wp['name']}  x={wp['x']}  y={wp['y']}  yaw={wp['yaw']}", flush=True)

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                        rclpy.parameter.Parameter.Type.BOOL, True)])

    print("[mission_runner] Waiting for Nav2 to become active...", flush=True)
    nav.waitUntilNav2Active()
    print("[mission_runner] Nav2 active. Starting mission loop.", flush=True)

    loop = 0
    try:
        while True:
            loop += 1
            print(f"\n[mission_runner] ── Loop {loop} ──", flush=True)

            for i, wp in enumerate(waypoints):
                name = wp["name"]
                pose = make_pose(nav, wp["x"], wp["y"], wp["yaw"])

                print(f"[mission_runner] → Navigating to '{name}' ({i+1}/{len(waypoints)})", flush=True)
                nav.goToPose(pose)

                while not nav.isTaskComplete():
                    feedback = nav.getFeedback()
                    if feedback:
                        cp = feedback.current_pose.pose
                        print(
                            f"  [{name}] x={cp.position.x:.2f}  y={cp.position.y:.2f}",
                            end="\r", flush=True
                        )

                result = nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f"\n[mission_runner] ✓ Reached '{name}'", flush=True)
                    delay = wp.get("delay", 0)
                    if delay and delay > 0:
                        print(f"[mission_runner] ⏳ Waiting {delay}s at '{name}'...", flush=True)
                        time.sleep(delay)
                elif result == TaskResult.CANCELED:
                    print(f"\n[mission_runner] Mission canceled.", flush=True)
                    nav.lifecycleShutdown()
                    return
                elif result == TaskResult.FAILED:
                    print(f"\n[mission_runner] ✗ Failed to reach '{name}', skipping.", flush=True)

    except KeyboardInterrupt:
        print("\n[mission_runner] Stopped by user.", flush=True)
        nav.cancelTask()

    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()