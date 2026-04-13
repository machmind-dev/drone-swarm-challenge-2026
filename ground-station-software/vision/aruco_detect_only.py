#!/usr/bin/env python3

import cv2
import numpy as np
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ArucoDetectOnlyNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_only')

        self.declare_parameter('image_topic', '/drone_2/camera/image_raw')
        self.declare_parameter('debug_topic', '/drone_2/aruco_debug')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('min_marker_px', 12.0)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.min_marker_px = float(self.get_parameter('min_marker_px').value)

        dict_name = str(self.get_parameter('aruco_dict').value)
        self.aruco_dict = self._get_aruco_dictionary(dict_name)

        # OpenCV 4.6 legacy API
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.frame_counter = 0
        self.last_ids = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos
        )

        self.pub = self.create_publisher(
            Image,
            self.debug_topic,
            10
        )

        self.get_logger().info(f'Listening on {self.image_topic}')
        self.get_logger().info(f'Publishing debug to {self.debug_topic}')
        self.get_logger().info(f'Using dictionary {dict_name}')
        self.get_logger().info(f'OpenCV version: {cv2.__version__}')

    def _get_aruco_dictionary(self, dict_name: str):
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

    def image_cb(self, msg: Image):
        try:
            self.frame_counter += 1

            # Log first frames to see what actually arrives
            if self.frame_counter <= 5:
                self.get_logger().info(
                    f"frame {self.frame_counter}: "
                    f"encoding={msg.encoding}, "
                    f"width={msg.width}, height={msg.height}, step={msg.step}, "
                    f"data_len={len(msg.data)}"
                )

            # Your ESP32 publishes mono8, so decode accordingly first
            if msg.encoding == 'mono8':
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if len(frame.shape) == 2:
                    gray = frame
                    frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                else:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.detector_params
            )

            detected_ids = []

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().tolist()
                filtered_corners = []
                filtered_ids = []

                for i, marker_id in enumerate(ids_flat):
                    c = corners[i].reshape(4, 2)
                    side_px = float(np.linalg.norm(c[0] - c[1]))

                    if side_px >= self.min_marker_px:
                        filtered_corners.append(corners[i])
                        filtered_ids.append(marker_id)

                        center = np.mean(c, axis=0).astype(int)
                        cv2.putText(
                            frame,
                            f'ID {marker_id} {side_px:.1f}px',
                            (int(center[0]) - 45, int(center[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            (0, 255, 0),
                            1
                        )

                if filtered_ids:
                    detected_ids = filtered_ids
                    cv2.aruco.drawDetectedMarkers(
                        frame,
                        filtered_corners,
                        np.array(filtered_ids, dtype=np.int32)
                    )

            if detected_ids != self.last_ids:
                if detected_ids:
                    self.get_logger().info(f'Detected IDs: {detected_ids}')
                else:
                    self.get_logger().info('No markers detected')
                self.last_ids = detected_ids

            cv2.putText(
                frame,
                f'Detected: {detected_ids if detected_ids else "none"}',
                (10, 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1
            )

            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'image_cb exception: {e}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectOnlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
