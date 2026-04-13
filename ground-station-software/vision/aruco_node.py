#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


@dataclass
class MarkerWorldPose:
    marker_id: int
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0


@dataclass
class DroneState:
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    camera_info_received: bool = False


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.declare_parameter('drone_names', ['drone_2'])
        self.declare_parameter('marker_size_m', 0.15)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('camera_frame_suffix', 'camera_optical_frame')
        self.declare_parameter('world_frame', 'map')

        self.declare_parameter('camera_to_body_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_to_body_rpy_deg', [0.0, 0.0, 0.0])

        # Format: "id,x,y,z,yaw_deg"
        self.declare_parameter('marker_map', [
            "0,0.0,0.0,0.0,0.0",
        ])

        self.drone_names: List[str] = list(self.get_parameter('drone_names').value)
        self.marker_size_m: float = float(self.get_parameter('marker_size_m').value)
        self.publish_debug_image: bool = bool(self.get_parameter('publish_debug_image').value)
        self.world_frame: str = str(self.get_parameter('world_frame').value)
        self.camera_frame_suffix: str = str(self.get_parameter('camera_frame_suffix').value)

        marker_map_raw = self.get_parameter('marker_map').value
        self.marker_map = self._parse_marker_map(marker_map_raw)

        cam_to_body_xyz = np.array(
            self.get_parameter('camera_to_body_xyz').value,
            dtype=np.float64
        )
        cam_to_body_rpy_deg = np.array(
            self.get_parameter('camera_to_body_rpy_deg').value,
            dtype=np.float64
        )
        self.T_cam_body = self._make_transform_xyz_rpy(
            cam_to_body_xyz,
            np.deg2rad(cam_to_body_rpy_deg)
        )

        dict_name = str(self.get_parameter('aruco_dict').value)
        self.aruco_dict = self._get_aruco_dictionary(dict_name)

        # OpenCV 4.6 compatible API
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.states: Dict[str, DroneState] = {}
        self.pose_pubs = {}
        self.marker_pubs = {}
        self.debug_pubs = {}

        for drone in self.drone_names:
            self.states[drone] = DroneState()

            self.create_subscription(
                CameraInfo,
                f'/{drone}/camera/camera_info',
                lambda msg, d=drone: self.camera_info_cb(msg, d),
                qos
            )

            self.create_subscription(
                Image,
                f'/{drone}/camera/image_raw',
                lambda msg, d=drone: self.image_cb(msg, d),
                qos
            )

            self.pose_pubs[drone] = self.create_publisher(
                PoseStamped,
                f'/{drone}/vision_pose',
                10
            )

            self.marker_pubs[drone] = self.create_publisher(
                MarkerArray,
                f'/{drone}/aruco_markers',
                10
            )

            if self.publish_debug_image:
                self.debug_pubs[drone] = self.create_publisher(
                    Image,
                    f'/{drone}/aruco_debug',
                    10
                )

        self.get_logger().info(f'ArucoNode started for drones: {self.drone_names}')
        self.get_logger().info(f'Marker map IDs: {sorted(self.marker_map.keys())}')
        self.get_logger().info(f'OpenCV version: {cv2.__version__}')

    def _parse_marker_map(self, raw) -> Dict[int, MarkerWorldPose]:
        parsed: Dict[int, MarkerWorldPose] = {}

        if not isinstance(raw, (list, tuple)):
            raise ValueError(
                "marker_map must be a list of strings like 'id,x,y,z,yaw_deg'"
            )

        for entry in raw:
            if not isinstance(entry, str):
                raise ValueError(f"marker_map entry must be string, got: {type(entry)}")

            parts = [p.strip() for p in entry.split(',')]
            if len(parts) not in (4, 5):
                raise ValueError(
                    f"Invalid marker_map entry '{entry}'. "
                    "Expected 'id,x,y,z' or 'id,x,y,z,yaw_deg'"
                )

            marker_id = int(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            yaw_deg = float(parts[4]) if len(parts) == 5 else 0.0

            parsed[marker_id] = MarkerWorldPose(
                marker_id=marker_id,
                x=x,
                y=y,
                z=z,
                yaw_deg=yaw_deg
            )

        return parsed

    def _get_aruco_dictionary(self, dict_name: str):
        if not hasattr(cv2.aruco, dict_name):
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))

    def _make_transform_xyz_rpy(self, xyz: np.ndarray, rpy_rad: np.ndarray) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R.from_euler('xyz', rpy_rad).as_matrix()
        T[:3, 3] = xyz
        return T

    def _rt_to_transform(self, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3], _ = cv2.Rodrigues(rvec)
        T[:3, 3] = tvec.reshape(3)
        return T

    def _transform_to_pose_msg(self, T: np.ndarray, stamp, frame_id: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(T[0, 3])
        msg.pose.position.y = float(T[1, 3])
        msg.pose.position.z = float(T[2, 3])

        quat = R.from_matrix(T[:3, :3]).as_quat()
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        return msg

    def marker_corners_local(self, size_m: float) -> np.ndarray:
        s = size_m / 2.0
        return np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float32)

    def marker_corners_world(self, marker_pose: MarkerWorldPose) -> np.ndarray:
        local = self.marker_corners_local(self.marker_size_m)
        yaw = math.radians(marker_pose.yaw_deg)

        rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw),  math.cos(yaw), 0.0],
            [0.0,            0.0,           1.0]
        ], dtype=np.float64)

        world = (rz @ local.T).T
        world[:, 0] += marker_pose.x
        world[:, 1] += marker_pose.y
        world[:, 2] += marker_pose.z
        return world.astype(np.float32)

    def camera_info_cb(self, msg: CameraInfo, drone: str):
        state = self.states[drone]
        state.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        state.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        state.camera_info_received = True
        self.get_logger().info(f'[{drone}] camera_info received')

    def image_cb(self, msg: Image, drone: str):
        state = self.states[drone]

        # Temporary fallback until ESP32 publishes camera_info
        if not state.camera_info_received:
            state.camera_matrix = np.array([
                [140.0, 0.0, 80.0],
                [0.0, 140.0, 60.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            state.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
            state.camera_info_received = True
            self.get_logger().warn(
                f'[{drone}] camera_info missing, using fallback intrinsics'
            )

        try:
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
        except Exception as e:
            self.get_logger().error(f'[{drone}] cv_bridge failed: {e}')
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params
        )

        marker_array = MarkerArray()

        if rejected is not None and len(rejected) > 0:
            cv2.aruco.drawDetectedMarkers(frame, rejected, borderColor=(0, 0, 255))

        if ids is None or len(ids) == 0:
            self.get_logger().info(f'[{drone}] no markers detected')
            self._publish_deleteall_markers(drone, msg.header, marker_array)
            self._publish_debug_if_needed(drone, frame, msg.header)
            return

        ids = ids.flatten().tolist()
        self.get_logger().info(f'[{drone}] detected IDs: {ids}')

        object_points = []
        image_points = []

        for idx, marker_id in enumerate(ids):
            c = corners[idx].reshape(4, 2).astype(np.float32)
            side_px = float(np.linalg.norm(c[0] - c[1]))

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

            if marker_id in self.marker_map:
                obj = self.marker_corners_world(self.marker_map[marker_id])
                object_points.append(obj)
                image_points.append(c)

            self._append_marker_visual(
                marker_array,
                msg.header,
                drone,
                marker_id
            )

        self.marker_pubs[drone].publish(marker_array)

        cv2.aruco.drawDetectedMarkers(
            frame,
            corners,
            np.array(ids, dtype=np.int32),
            borderColor=(0, 255, 0)
        )

        if len(object_points) == 0:
            self.get_logger().warn(f'[{drone}] markers detected, but none matched marker_map')
            self._publish_debug_if_needed(drone, frame, msg.header)
            return

        object_points = np.concatenate(object_points, axis=0).reshape(-1, 3)
        image_points = np.concatenate(image_points, axis=0).reshape(-1, 2)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            state.camera_matrix,
            state.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            self.get_logger().warn(f'[{drone}] solvePnP failed')
            self._publish_debug_if_needed(drone, frame, msg.header)
            return

        # solvePnP gives transform world -> camera
        t_cam_world = self._rt_to_transform(rvec, tvec)
        t_world_cam = np.linalg.inv(t_cam_world)

        # Optional rigid mount transform from camera -> drone body
        t_world_body = t_world_cam @ self.T_cam_body

        pose_msg = self._transform_to_pose_msg(
            t_world_body,
            msg.header.stamp,
            self.world_frame
        )
        self.pose_pubs[drone].publish(pose_msg)

        cv2.drawFrameAxes(
            frame,
            state.camera_matrix,
            state.dist_coeffs,
            rvec,
            tvec,
            0.15
        )

        xyz = t_world_body[:3, 3]
        text = f"{drone} xyz=({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) m"
        cv2.putText(
            frame,
            text,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        self._publish_debug_if_needed(drone, frame, msg.header)

    def _publish_deleteall_markers(self, drone: str, header, marker_array: MarkerArray):
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.ns = f'{drone}_aruco'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pubs[drone].publish(marker_array)

    def _append_marker_visual(
        self,
        marker_array: MarkerArray,
        header,
        drone: str,
        marker_id: int
    ):
        if marker_id not in self.marker_map:
            return

        wp = self.marker_map[marker_id]

        cube = Marker()
        cube.header = header
        cube.header.frame_id = self.world_frame
        cube.ns = f'{drone}_aruco_known'
        cube.id = int(marker_id)
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = wp.x
        cube.pose.position.y = wp.y
        cube.pose.position.z = wp.z

        q = R.from_euler('z', math.radians(wp.yaw_deg)).as_quat()
        cube.pose.orientation.x = float(q[0])
        cube.pose.orientation.y = float(q[1])
        cube.pose.orientation.z = float(q[2])
        cube.pose.orientation.w = float(q[3])

        cube.scale.x = self.marker_size_m
        cube.scale.y = self.marker_size_m
        cube.scale.z = 0.02
        cube.color = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.6)

        text = Marker()
        text.header = header
        text.header.frame_id = self.world_frame
        text.ns = f'{drone}_aruco_text'
        text.id = int(1000 + marker_id)
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = wp.x
        text.pose.position.y = wp.y
        text.pose.position.z = wp.z + 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.20
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f'ID {marker_id}'

        marker_array.markers.append(cube)
        marker_array.markers.append(text)

    def _publish_debug_if_needed(self, drone: str, frame: np.ndarray, header):
        if not self.publish_debug_image:
            return
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = header
        self.debug_pubs[drone].publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
