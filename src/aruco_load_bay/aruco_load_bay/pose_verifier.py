#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Empty
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import math
import numpy as np

class PoseVerifierNode(Node):
    def __init__(self):
        super().__init__('pose_verifier')

        # Tolerances & Target Parameters
        self.declare_parameter('target_distance', 1.00)   # Target standoff distance: 1.00 m
        self.declare_parameter('x_tolerance', 0.05)       # +-0.05 m (lateral error)
        self.declare_parameter('y_tolerance', 0.05)       # +-0.05 m (forward distance error)
        self.declare_parameter('yaw_tolerance_deg', 5.0)  # +-5.0 degrees (yaw error)
        self.declare_parameter('camera_frame', 'camera_link_optical')

        self.target_dist = float(self.get_parameter('target_distance').value)
        self.x_tol = float(self.get_parameter('x_tolerance').value)
        self.y_tol = float(self.get_parameter('y_tolerance').value)
        self.yaw_tol_deg = float(self.get_parameter('yaw_tolerance_deg').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/pose_verification/status', 10)
        self.pub_marker_array = self.create_publisher(MarkerArray, '/pose_verification/rviz_marker', 10)
        self.pub_dock_verified = self.create_publisher(Bool, '/dock_pose_verified', 10)

        # Subscribers & Services for reset/trigger
        self.sub_reset = self.create_subscription(Empty, '/pose_verification/reset', self.reset_cb, 10)
        self.sub_trigger_bool = self.create_subscription(Bool, '/pose_verification/trigger', self.reset_cb, 10)
        self.srv_trigger = self.create_service(Trigger, '/pose_verification/trigger', self.trigger_service_cb)

        # States: 'APPROACHING' -> 'ALIGNING' -> 'POSE_VERIFIED' | 'LOST_MARKER'
        self.state = 'APPROACHING'
        self.is_passed = False
        self.pass_hold_ticks = 0
        self.tf_lost_ticks = 0

        self.last_dx = 0.0
        self.last_dy = 0.0
        self.last_dyaw = 0.0
        self.last_log_time = 0.0

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Pose Verifier Node initialized.')
        self.get_logger().info(f'Tolerances: X: +- {self.x_tol:.2f}m | Y (Distance): +- {self.y_tol:.2f}m | Yaw: +- {self.yaw_tol_deg:.1f} deg')

    def reset_cb(self, msg=None):
        self.state = 'APPROACHING'
        self.is_passed = False
        self.pass_hold_ticks = 0
        self.tf_lost_ticks = 0
        self.get_logger().info('[VERIFY] Resetting Pose Verifier State Machine to APPROACHING...')

    def trigger_service_cb(self, request, response):
        self.reset_cb()
        response.success = True
        response.message = "Docking and Pose Verification triggered!"
        return response

    def control_loop(self):
        cmd = Twist()
        status_msg = String()
        now_sec = self.get_clock().now().nanoseconds / 1e9

        try:
            # Look up transform from camera_frame (camera_link_optical) to aruco_marker_detected
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                'aruco_marker_detected',
                rclpy.time.Time()
            )

            # Transform found: reset lost tick counter
            self.tf_lost_ticks = 0

            # Camera REP-104 Optical Frame:
            # x_cam: Lateral error delta_X (right positive)
            # z_cam: Forward distance to ArUco marker
            x_cam = t.transform.translation.x
            z_cam = t.transform.translation.z

            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            # Calculate Yaw error from quaternion
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = math.degrees(yaw)

            # Calculated Pose Errors
            dX = x_cam
            dZ = z_cam - self.target_dist
            dYaw = yaw_deg

            self.last_dx = dX
            self.last_dy = dZ
            self.last_dyaw = dYaw

            # Individual Tolerance Evaluation
            x_pass = abs(dX) <= self.x_tol
            y_pass = abs(dZ) <= self.y_tol
            yaw_pass = abs(dYaw) <= self.yaw_tol_deg

            all_passed = x_pass and y_pass and yaw_pass

            # Debounce / Hold Hysteresis: Require ALL conditions continuously valid for at least 0.5s (10 ticks at 20Hz)
            if all_passed:
                self.pass_hold_ticks += 1
            else:
                if self.state != 'POSE_VERIFIED':
                    self.pass_hold_ticks = 0

            # State Transition to POSE_VERIFIED once 0.5s continuous validity is reached
            if self.pass_hold_ticks >= 10:
                self.state = 'POSE_VERIFIED'

            # ---------------------------------------------------
            # STATE MACHINE HANDLING
            # ---------------------------------------------------
            if self.state == 'POSE_VERIFIED':
                self.is_passed = True

                # LOCK VELOCITY TO ZERO (Prevent robot movement after POSE_VERIFIED)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd_vel.publish(cmd)

                # Publish confirmation topic
                b_msg = Bool()
                b_msg.data = True
                self.pub_dock_verified.publish(b_msg)

                status_str = f"STATUS: POSE VERIFIED\nX: PASS | Y: PASS | YAW: PASS\ndX: {dX:+.3f}m | dZ: {dZ:+.3f}m | dYaw: {dYaw:+.1f} deg"
                status_msg.data = status_str
                self.publish_rviz_3d_text('POSE VERIFIED / PASS', dX, dZ, dYaw, state='POSE_VERIFIED', is_passed=True)

                if now_sec - self.last_log_time >= 1.0:
                    self.last_log_time = now_sec
                    self.get_logger().info("[ARUCO] Marker detected")
                    self.get_logger().info(f"[VERIFY] X error:   {dX:+.3f} m -> PASS")
                    self.get_logger().info(f"[VERIFY] Y error:   {dZ:+.3f} m -> PASS")
                    self.get_logger().info(f"[VERIFY] Yaw error: {dYaw:+.1f} deg -> PASS")
                    self.get_logger().info("[VERIFY] STATUS: POSE_VERIFIED")

            else:
                # Outside tolerance: Determine state (APPROACHING vs ALIGNING)
                if not y_pass:
                    self.state = 'APPROACHING'
                else:
                    self.state = 'ALIGNING'

                self.is_passed = False

                # Proportional Velocity Controller
                k_p_lin = 0.45
                k_p_lat = 0.75
                k_p_yaw = 0.015

                if not y_pass:
                    if dZ > 0:
                        v_x = float(np.clip(max(k_p_lin * dZ, 0.07), 0.07, 0.18))
                    else:
                        v_x = float(np.clip(min(k_p_lin * dZ, -0.07), -0.18, -0.07))
                else:
                    v_x = 0.0

                w_z = float(np.clip(-k_p_lat * dX - k_p_yaw * dYaw, -0.25, 0.25))

                cmd.linear.x = v_x
                cmd.angular.z = w_z
                self.pub_cmd_vel.publish(cmd)

                status_str = f"STATUS: {self.state}...\nX: {'PASS' if x_pass else 'FAIL'} | Y: {'PASS' if y_pass else 'FAIL'} | YAW: {'PASS' if yaw_pass else 'FAIL'}\ndX: {dX:+.3f}m | dZ: {dZ:+.3f}m | dYaw: {dYaw:+.1f} deg"
                status_msg.data = status_str
                self.publish_rviz_3d_text(f"{self.state}...", dX, dZ, dYaw, state=self.state, is_passed=False)

                if now_sec - self.last_log_time >= 1.0:
                    self.last_log_time = now_sec
                    self.get_logger().info("[ARUCO] Marker detected")
                    self.get_logger().info(f"[VERIFY] X error:   {dX:+.3f} m -> {'PASS' if x_pass else 'FAIL'}")
                    self.get_logger().info(f"[VERIFY] Y error:   {dZ:+.3f} m -> {'PASS' if y_pass else 'FAIL'}")
                    self.get_logger().info(f"[VERIFY] Yaw error: {dYaw:+.1f} deg -> {'PASS' if yaw_pass else 'FAIL'}")
                    self.get_logger().info(f"[VERIFY] STATUS: {self.state}")

        except TransformException as ex:
            self.tf_lost_ticks += 1
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd_vel.publish(cmd)

            # Wait at least 1.0 second (20 ticks at 20Hz) before declaring LOST_MARKER
            if self.tf_lost_ticks >= 20:
                if self.state != 'POSE_VERIFIED':
                    self.state = 'LOST_MARKER'
                status_msg.data = f"STATUS: {self.state}"
                self.publish_rviz_3d_text(self.state, self.last_dx, self.last_dy, self.last_dyaw, state=self.state, is_passed=(self.state == 'POSE_VERIFIED'))
            else:
                # Maintain current stable display during temporary TF jitter
                if self.state == 'POSE_VERIFIED':
                    status_msg.data = f"STATUS: POSE VERIFIED\nX: PASS | Y: PASS | YAW: PASS\ndX: {self.last_dx:+.3f}m | dZ: {self.last_dy:+.3f}m | dYaw: {self.last_dyaw:+.1f} deg"
                    self.publish_rviz_3d_text('POSE VERIFIED / PASS', self.last_dx, self.last_dy, self.last_dyaw, state='POSE_VERIFIED', is_passed=True)
                else:
                    status_msg.data = f"STATUS: {self.state}..."
                    self.publish_rviz_3d_text(f"{self.state}...", self.last_dx, self.last_dy, self.last_dyaw, state=self.state, is_passed=False)

        if status_msg.data:
            self.pub_status.publish(status_msg)

    def publish_rviz_3d_text(self, state_str, dX, dZ, dYaw, state='APPROACHING', is_passed=False):
        ma = MarkerArray()
        now_stamp = self.get_clock().now().to_msg()
        m_lifetime = Duration(seconds=0.1).to_msg()

        # Determine color scheme based on state & verification outcome
        if is_passed or state == 'POSE_VERIFIED':
            # BRIGHT GREEN for POSE VERIFIED / PASS
            r_title, g_title, b_title = 0.0, 1.0, 0.0
            r_vals,  g_vals,  b_vals  = 0.3, 1.0, 0.3
            r_cube,  g_cube,  b_cube  = 0.0, 1.0, 0.0
        elif state == 'LOST_MARKER':
            # RED for LOST MARKER
            r_title, g_title, b_title = 1.0, 0.0, 0.0
            r_vals,  g_vals,  b_vals  = 1.0, 0.3, 0.3
            r_cube,  g_cube,  b_cube  = 1.0, 0.0, 0.0
        else:
            # AMBER/ORANGE for APPROACHING / ALIGNING
            r_title, g_title, b_title = 1.0, 0.5, 0.0
            r_vals,  g_vals,  b_vals  = 1.0, 0.8, 0.2
            r_cube,  g_cube,  b_cube  = 1.0, 0.4, 0.0

        # 1. Main Title Text Marker ("APPROACHING...", "POSE VERIFIED / PASS")
        m_title = Marker()
        m_title.header.frame_id = "aruco_marker_detected"
        m_title.header.stamp = now_stamp
        m_title.ns = "alignment_text"
        m_title.id = 0
        m_title.type = Marker.TEXT_VIEW_FACING
        m_title.action = Marker.ADD
        m_title.lifetime = m_lifetime
        m_title.pose.position.x = 0.0
        m_title.pose.position.y = -0.35
        m_title.pose.position.z = 0.0
        m_title.scale.z = 0.18
        m_title.color.r = float(r_title)
        m_title.color.g = float(g_title)
        m_title.color.b = float(b_title)
        m_title.color.a = 1.0
        m_title.text = state_str
        ma.markers.append(m_title)

        # 2. Precision Values Text Marker ("[dX=...m | dZ=...m | dYaw=...deg]")
        m_vals = Marker()
        m_vals.header.frame_id = "aruco_marker_detected"
        m_vals.header.stamp = now_stamp
        m_vals.ns = "alignment_values"
        m_vals.id = 1
        m_vals.type = Marker.TEXT_VIEW_FACING
        m_vals.action = Marker.ADD
        m_vals.lifetime = m_lifetime
        m_vals.pose.position.x = 0.0
        m_vals.pose.position.y = -0.55
        m_vals.pose.position.z = 0.0
        m_vals.scale.z = 0.12
        m_vals.color.r = float(r_vals)
        m_vals.color.g = float(g_vals)
        m_vals.color.b = float(b_vals)
        m_vals.color.a = 1.0
        m_vals.text = f"[dX={dX:+.3f}m | dZ={dZ:+.3f}m | dYaw={dYaw:+.1f} deg]"
        ma.markers.append(m_vals)

        # 3. Target Docking Cube Marker
        m_cube = Marker()
        m_cube.header.frame_id = "aruco_marker_detected"
        m_cube.header.stamp = now_stamp
        m_cube.ns = "target_cube"
        m_cube.id = 2
        m_cube.type = Marker.CUBE
        m_cube.action = Marker.ADD
        m_cube.lifetime = m_lifetime
        m_cube.pose.position.x = 0.0
        m_cube.pose.position.y = 0.0
        m_cube.pose.position.z = 0.0
        m_cube.pose.orientation.w = 1.0
        m_cube.scale.x = 0.28
        m_cube.scale.y = 0.28
        m_cube.scale.z = 0.28
        m_cube.color.r = float(r_cube)
        m_cube.color.g = float(g_cube)
        m_cube.color.b = float(b_cube)
        m_cube.color.a = 0.90
        ma.markers.append(m_cube)

        self.pub_marker_array.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = PoseVerifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
