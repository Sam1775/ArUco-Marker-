#!/bin/bash
# Script to build and launch the ArUco Load-Bay Pose Verification Simulation

set -e

echo "=== Cleaning Up Previous Simulation Processes ==="
killall -9 gz sim parameter_bridge robot_state_publisher rviz2 aruco_detector pose_verifier 2>/dev/null || true
sleep 1

echo "=== Sourcing ROS 2 Jazzy ==="
source /opt/ros/jazzy/setup.bash

echo "=== Building Workspace ==="
cd /home/sam_17/aruco_pose_ws
colcon build --symlink-install --packages-select aruco_load_bay

echo "=== Sourcing Workspace Overlay ==="
source install/setup.bash

echo "=== Exporting Environment & Model Paths ==="
export GZ_PARTITION=load_bay_sim
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/sam_17/aruco_pose_ws/src/aruco_load_bay/models

echo "=== Launching Simulation ==="
ros2 launch aruco_load_bay load_bay_sim.launch.py
