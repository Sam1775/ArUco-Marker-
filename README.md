# ArUco Load-Bay Pose Verification

**Marker-referenced pose error checking and dwell-windowed docking verification for autonomous mobile robots — built and integration-tested in ROS 2 Jazzy.**

---

## Overview

Standard AMR navigation stacks (Nav2 + AMCL) localize a robot to roughly **±5–10 cm** — reliable for corridor travel, but nowhere near tight enough for a robot to physically interface with a pallet bay, charging contact, or conveyor. Wheel odometry drift compounds the problem over distance.

This package solves the "last-centimeter" problem by decoupling terminal docking from global navigation: instead of trusting the robot's own (potentially drifted) pose estimate, the robot verifies its position against a **fixed ArUco marker** at the dock, using computer vision and TF2 — not odometry.

A robot is only declared **docked** once its pose error against the marker holds within tight tolerance for a sustained window of checks, not a single lucky frame.

---

## How It Works

```
        ┌────────────────────┐         ┌───────────────────────┐
        │  Camera Stream      │         │  Robot TF Tree          │
        │  /image_raw           │         │  base_link → camera_link │
        │  /camera_info          │         └───────────┬─────────────┘
        └──────────┬───────────┘                     │
                   │                                    │
                   ▼                                    │
        ┌─────────────────────────┐                  │
        │  aruco_pose_publisher       │                  │
        │  • Detect marker (cv2.aruco)  │                  │
        │  • solvePnP pose estimate     │                  │
        │  • Broadcast TF: camera →     │                  │
        │    dock_marker                │                  │
        └──────────┬───────────────┘                  │
                   │  TF: dock_marker                     │
                   ▼                                    ▼
        ┌───────────────────────────────────────────────┐
        │  pose_verification_node                            │
        │  • Lookup TF: base_link → dock_marker             │
        │  • Compute ΔX, ΔY, ΔYaw vs. target standoff       │
        │  • Check tolerance + dwell window                 │
        │  • Publish /dock_verified, /dock_pose_error       │
        │  • Publish color-coded RViz marker                │
        └───────────────────────────────────────────────┘
```

**Verification logic:**

```python
delta_X   = marker_x_in_base_frame
delta_Y   = marker_y_in_base_frame - target_standoff_y
delta_Yaw = yaw(marker_in_base_frame)

in_tolerance = (abs(delta_X)   <= tolerance_x   and
                abs(delta_Y)   <= tolerance_y   and
                abs(delta_Yaw) <= tolerance_yaw_deg)

consecutive_count = consecutive_count + 1 if in_tolerance else 0
verified = consecutive_count >= dwell_window
```

The **dwell window** (default: 10 consecutive in-tolerance checks) exists because a single well-aligned camera frame isn't proof of a stable dock — monocular pose estimation is noisiest at oblique viewing angles, and requiring sustained agreement filters that noise out before declaring success.

---

## Features

- 🎯 **ArUco marker detection** — `cv2.aruco.ArucoDetector` with configurable dictionary/ID
- 📐 **6-DoF pose estimation** — `solvePnP` (IPPE_SQUARE) for planar square markers
- 🔗 **TF2-native pose error** — no raw pixel-offset hacks, error computed through the robot's real transform tree
- ⏱️ **Dwell-window verification** — rejects false positives from single-frame noise
- 🎨 **Live RViz visualization** — color-coded status marker (🔴 not visible → 🟠 out of tolerance → 🟡 converging → 🟢 verified)
- ⚙️ **Fully parameterized** — every tolerance, frame name, and rate lives in `params.yaml`, nothing hardcoded

---

## Project Structure

```
aruco_dock_verification/
├── aruco_dock_verification/
│   ├── aruco_pose_publisher.py     # ArUco detection + TF pose broadcast
│   └── pose_verification_node.py   # Pose error + tolerance + dwell window
├── launch/
│   └── verification_demo.launch.py
├── config/
│   └── params.yaml
├── rviz/
│   └── verification.rviz
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Installation

```bash
# Clone into your workspace
cd ~/your_ws/src
git clone <your-repo-url> aruco_dock_verification

# Build
cd ~/your_ws
colcon build --packages-select aruco_dock_verification
source install/setup.bash
```

## Usage

```bash
# Launch detection + verification + RViz
ros2 launch aruco_dock_verification verification_demo.launch.py

# Without RViz
ros2 launch aruco_dock_verification verification_demo.launch.py rviz:=false

# With a custom params file
ros2 launch aruco_dock_verification verification_demo.launch.py \
    params_file:=/path/to/custom_params.yaml
```

Monitor status directly:

```bash
ros2 topic echo /dock_verified
ros2 topic echo /dock_pose_error
```

---

## Configuration

All tunables live in [`config/params.yaml`](config/params.yaml):

| Parameter | Default | Description |
|---|---|---|
| `marker_id` | `0` | ArUco marker ID to track |
| `marker_dictionary` | `DICT_4X4_50` | Marker dictionary |
| `marker_size_m` | `0.10` | Physical marker side length (m) |
| `target_standoff_y` | `0.20` | Desired final gap to the marker (m) |
| `tolerance_x` / `tolerance_y` | `0.03` | Forward / lateral tolerance (m) |
| `tolerance_yaw_deg` | `1.5` | Heading tolerance (degrees) |
| `dwell_window` | `10` | Consecutive in-tolerance checks required |
| `check_rate_hz` | `5.0` | Verification loop rate |

---

## Topics

| Topic | Type | Description |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | Input camera stream (subscribed) |
| `/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (subscribed) |
| `/aruco_debug_image` | `sensor_msgs/Image` | Detection overlay (published) |
| `/dock_verified` | `std_msgs/Bool` | Final docking status (published) |
| `/dock_pose_error` | `geometry_msgs/Vector3` | Live ΔX, ΔY, ΔYaw (published) |
| `/dock_verification_marker` | `visualization_msgs/Marker` | RViz status overlay (published) |

---

## Real-World Applications

The same **detect → measure → verify** pattern applies wherever a mobile robot must physically interface with fixed infrastructure:

- Warehouse pallet pick/drop
- EV / battery charging docks
- Conveyor and cross-dock interfacing
- Hospital and facility delivery robots
- Automated trailer loading
- Precision assembly stations

---

## Known Issues & Fixes

| Issue | Root Cause | Fix |
|---|---|---|
| RViz "message queue full" for `dock_marker` | No camera feed connected — marker never detected, TF never broadcast | Expected without a live camera; reduce `check_rate_hz` for bench testing |
| `RCLError: rcl_shutdown already called` on `Ctrl+C` | Launch system and node's own `finally` block both called `rclpy.shutdown()` | Guarded with `if rclpy.ok(): rclpy.shutdown()` in both nodes |

---

## Roadmap

- [x] ArUco detection + TF pose broadcast
- [x] Pose error + dwell-window verification logic
- [x] RViz color-coded status visualization
- [x] Clean shutdown handling
- [ ] End-to-end validation against a live camera feed
- [ ] Integration with a full Nav2 pre-dock approach sequence

---

## Author

**Sam** — Robotics & Automation Engineering
Built as part of a warehouse AMR docking verification project.

## License

Apache 2.0 — see [`LICENSE`](LICENSE) for details.
