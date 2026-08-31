#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import os

class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Parameters
        self.declare_parameter('marker_size', 0.35) # 35cm marker
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('target_frame', 'aruco_marker_detected')
        
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        
        # Ensure marker texture image exists on disk
        self._ensure_marker_texture()

        # OpenCV ArUco Setup
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector_params = aruco.DetectorParameters()
        self.detector_params.minMarkerPerimeterRate = 0.01
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23
        self.detector_params.adaptiveThreshWinSizeStep = 5

        if hasattr(aruco, 'ArucoDetector'):
            self.detector = aruco.ArucoDetector(self.dictionary, self.detector_params)
        else:
            self.detector = None

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscriptions using SensorDataQoS for Gazebo bridge compatibility
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.status_text = "STATUS: SEARCHING..."
        self.sub_status = self.create_subscription(
            String,
            '/pose_verification/status',
            self.status_callback,
            10
        )

        # Publisher for annotated debug visual feed with SensorDataQoS for RViz
        annotated_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub_annotated = self.create_publisher(Image, '/aruco/image_annotated', annotated_qos)
        self.get_logger().info('ArUco Detector Node initialized.')

    def status_callback(self, msg: String):
        self.status_text = msg.data

    def _ensure_marker_texture(self):
        tex_path = '/home/sam_17/aruco_pose_ws/src/aruco_load_bay/models/load_bay_marker/materials/textures/marker_0.png'
        if not os.path.exists(tex_path):
            try:
                os.makedirs(os.path.dirname(tex_path), exist_ok=True)
                dict_4x4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
                img = aruco.generateImageMarker(dict_4x4, 0, 512)
                full = np.ones((640, 640), dtype=np.uint8) * 255
                full[64:576, 64:576] = img
                cv2.imwrite(tex_path, full)
                self.get_logger().info(f'Generated ArUco texture at {tex_path}')
            except Exception as e:
                self.get_logger().warn(f'Could not generate texture: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera calibration parameters received.')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        annotated_image = cv_image.copy()

        # If camera calibration is not yet received, use estimated focal params
        if self.camera_matrix is None:
            h, w = cv_image.shape[:2]
            focal_length = w
            self.camera_matrix = np.array([[focal_length, 0, w / 2],
                                           [0, focal_length, h / 2],
                                           [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # Detect markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)

        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                c = corners[i]
                aruco.drawDetectedMarkers(annotated_image, corners, ids)

                # Solve 3D Pose using solvePnP for robustness
                half_size = self.marker_size / 2.0
                obj_points = np.array([
                    [-half_size,  half_size, 0],
                    [ half_size,  half_size, 0],
                    [ half_size, -half_size, 0],
                    [-half_size, -half_size, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    c[0],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    # Draw 3D coordinate axis on the marker
                    cv2.drawFrameAxes(annotated_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.75)

                    tx, ty, tz = tvec.ravel()
                    dist = np.sqrt(tx**2 + ty**2 + tz**2)

                    # Compute rotation matrix & quaternion
                    R, _ = cv2.Rodrigues(rvec)
                    qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

                    # Broadcast TF with current ROS sim clock
                    self.broadcast_tf(tx, ty, tz, qx, qy, qz, qw, self.get_clock().now().to_msg())

                    # Render HUD Overlay matching user reference UI
                    lines = self.status_text.split('\n')
                    y_pos = 35

                    if "PASS" in self.status_text:
                        text_color = (0, 255, 0)     # Green in BGR
                        box_border = (0, 200, 0)
                    elif "FAIL" in self.status_text:
                        text_color = (0, 0, 255)     # Red in BGR
                        box_border = (0, 0, 200)
                    else:
                        text_color = (0, 200, 255)   # Amber/Orange in BGR
                        box_border = (0, 140, 255)

                    for line in lines:
                        (text_w, text_h), baseline = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
                        cv2.rectangle(annotated_image, (15, y_pos - text_h - 4), (25 + text_w, y_pos + 6), (20, 20, 20), -1)
                        cv2.rectangle(annotated_image, (15, y_pos - text_h - 4), (25 + text_w, y_pos + 6), box_border, 1)
                        cv2.putText(annotated_image, line, (20, y_pos),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, text_color, 2)
                        y_pos += 32

        else:
            (text_w, text_h), baseline = cv2.getTextSize("STATUS: SEARCHING FOR ARUCO MARKER...", cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
            cv2.rectangle(annotated_image, (15, 35 - text_h - 4), (25 + text_w, 35 + 6), (20, 20, 20), -1)
            cv2.rectangle(annotated_image, (15, 35 - text_h - 4), (25 + text_w, 35 + 6), (0, 0, 255), 1)
            cv2.putText(annotated_image, "STATUS: SEARCHING FOR ARUCO MARKER...", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        # Publish debug HUD image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

    def broadcast_tf(self, tx, ty, tz, qx, qy, qz, qw, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.target_frame

        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectorNode()
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
