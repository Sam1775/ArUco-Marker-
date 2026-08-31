#!/bin/bash
# Script to trigger re-docking & pose verification after teleoperating the robot

echo "=== Triggering Autonomous Docking & Pose Verification ==="
source /opt/ros/jazzy/setup.bash
source /home/sam_17/aruco_pose_ws/install/setup.bash

ros2 topic pub --once /pose_verification/reset std_msgs/msg/Empty {}
echo "=== Docking Trigger Sent! Robot is aligning to ArUco Marker... ==="
