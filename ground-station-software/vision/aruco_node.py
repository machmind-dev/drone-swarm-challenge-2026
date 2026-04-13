#!/usr/bin/env python3
"""
aruco_node.py  —  Mach Mind GCS ArUco detection node
=====================================================
Runs on: Raspberry Pi 5 (ROS 2 Jazzy)

Load params from:
  ros2 run <pkg> aruco_node --ros-args --params-file aruco_params.yaml

Or directly:
  python3 aruco_node.py --ros-args --params-file aruco_params.yaml

Per-drone topics:
  Sub:  /{drone}/camera/image_raw        (mono8, ESP32S3)
  Sub:  /{drone}/camera/camera_info      (optional — fallback if missing)
  Pub:  /{drone}/vision_pose             (PoseStamped  → firmware)
  Pub:  /{drone}/aruco_markers           (MarkerArray  → RViz cubes + labels)
  Pub:  /{drone}/aruco_debug             (Image bgr8   → RViz Image display)

Changes vs original:
  - drone_names default covers all 5 drones
  - angle/distance logged per detected marker (H, V, dist)
  - angle/distance overlaid on debug image
  - all 5 drones handled from one node instance
  - scipy replaced with inline quaternion math (no extra dep)
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# ── Geometry helpers (no scipy dependency) ────────────────────────────────────

def euler_xyz_to_matrix(rpy_rad: np.ndarray) -> np.ndarray:
    """Roll-pitch-yaw (XYZ extrinsic) → 3×3 rotation matrix."""
    r, p, y = rpy_rad
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]], dtype=np.float64)
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]], dtype=np.float64)
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,            1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def matrix_to_quaternion(R: np.ndarray):
    """3×3 rotation matrix → (qx, qy, qz, qw)."""
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def yaw_to_matrix(yaw_deg: float) -> np.ndarray:
    """Yaw-only rotation matrix (around Z)."""
    y = math.radians(yaw_deg)
    return np.array([[math.cos(y), -math.sin(y), 0],
                     [math.sin(y),  math.cos(y), 0],
                     [0,            0,            1]], dtype=np.float64)


# ── Data classes ──────────────────────────────────────────────────────────────

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
    dist_coeffs:   Optional[np.ndarray] = None
    camera_info_received: bool = False


# ── Per-drone debug overlay colour ───────────────────────────────────────────

_PALETTE_BGR = [
    (0,   255,   0),   # drone_1 green
    (0,   200, 255),   # drone_2 yellow
    (255,  80,   0),   # drone_3 blue
    (0,    80, 255),   # drone_4 orange
    (200,   0, 255),   # drone_5 magenta
]

def _drone_index(drone_name: str) -> int:
    """drone_1 → 0, drone_2 → 1, etc."""
    try:
        return int(drone_name.split('_')[-1]) - 1
    except ValueError:
        return 0

def drone_color(drone_name: str):
    return _PALETTE_BGR[_drone_index(drone_name) % len(_PALETTE_BGR)]


# ── Main node ─────────────────────────────────────────────────────────────────

class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('drone_names',
                               ['drone_1', 'drone_2', 'drone_3', 'drone_4', 'drone_5'])
        self.declare_parameter('marker_size_m',          0.15)
        self.declare_parameter('aruco_dict',             'DICT_4X4_50')
        self.declare_parameter('publish_debug_image',    True)
        self.declare_parameter('camera_frame_suffix',    'camera_optical_frame')
        self.declare_parameter('world_frame',            'map')
        self.declare_parameter('camera_to_body_xyz',     [0.0, 0.0, 0.0])
        self.declare_parameter('camera_to_body_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('marker_map',             ['0,0.0,0.0,0.0,0.0'])

        self.drone_names: List[str] = list(
            self.get_parameter('drone_names').value)
        self.marker_size_m: float = float(
            self.get_parameter('marker_size_m').value)
        self.publish_debug: bool = bool(
            self.get_parameter('publish_debug_image').value)
        self.world_frame: str = str(
            self.get_parameter('world_frame').value)
        self.camera_frame_suffix: str = str(
            self.get_parameter('camera_frame_suffix').value)

        self.marker_map: Dict[int, MarkerWorldPose] = self._parse_marker_map(
            self.get_parameter('marker_map').value)

        cam_xyz = np.array(
            self.get_parameter('camera_to_body_xyz').value, dtype=np.float64)
        cam_rpy = np.deg2rad(np.array(
            self.get_parameter('camera_to_body_rpy_deg').value, dtype=np.float64))
        self.T_cam_body = self._make_transform(cam_xyz, cam_rpy)

        # ── ArUco — OpenCV 4.6 compatible ────────────────────────────────────
        dict_name = str(self.get_parameter('aruco_dict').value)
        self.aruco_dict = self._get_aruco_dict(dict_name)

        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.detector_params = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_params = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        # ── Per-drone subscribers + publishers ────────────────────────────────
        self.states:      Dict[str, DroneState] = {}
        self.pose_pubs:   Dict[str, any] = {}
        self.marker_pubs: Dict[str, any] = {}
        self.debug_pubs:  Dict[str, any] = {}

        for drone in self.drone_names:
            self.states[drone] = DroneState()

            self.create_subscription(
                CameraInfo,
                f'/{drone}/camera/camera_info',
                lambda msg, d=drone: self._camera_info_cb(msg, d),
                qos)

            self.create_subscription(
                Image,
                f'/{drone}/camera/image_raw',
                lambda msg, d=drone: self._image_cb(msg, d),
                qos)

            self.pose_pubs[drone] = self.create_publisher(
                PoseStamped, f'/{drone}/vision_pose', 10)

            self.marker_pubs[drone] = self.create_publisher(
                MarkerArray, f'/{drone}/aruco_markers', 10)

            if self.publish_debug:
                self.debug_pubs[drone] = self.create_publisher(
                    Image, f'/{drone}/aruco_debug', 10)

        self.get_logger().info(
            f'Mach Mind ArUco node — drones: {self.drone_names}')
        self.get_logger().info(
            f'Marker map IDs: {sorted(self.marker_map.keys())}')
        self.get_logger().info(
            f'Marker size: {self.marker_size_m} m | '
            f'Dict: {dict_name} | OpenCV: {cv2.__version__}')

    # ── Param parsing ─────────────────────────────────────────────────────────

    def _parse_marker_map(self, raw) -> Dict[int, MarkerWorldPose]:
        parsed: Dict[int, MarkerWorldPose] = {}
        for entry in raw:
            parts = [p.strip() for p in str(entry).split(',')]
            if len(parts) not in (4, 5):
                raise ValueError(
                    f"Bad marker_map entry '{entry}'. "
                    "Expected 'id,x,y,z' or 'id,x,y,z,yaw_deg'")
            mid     = int(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            yaw     = float(parts[4]) if len(parts) == 5 else 0.0
            parsed[mid] = MarkerWorldPose(mid, x, y, z, yaw)
        return parsed

    def _get_aruco_dict(self, name: str):
        if not hasattr(cv2.aruco, name):
            raise ValueError(f'Unknown ArUco dictionary: {name}')
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _make_transform(self, xyz: np.ndarray, rpy_rad: np.ndarray) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = euler_xyz_to_matrix(rpy_rad)
        T[:3, 3]  = xyz
        return T

    def _rt_to_transform(self, rvec, tvec) -> np.ndarray:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3], _ = cv2.Rodrigues(rvec)
        T[:3, 3]     = tvec.reshape(3)
        return T

    def _transform_to_pose(self, T: np.ndarray, stamp, frame_id: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(T[0, 3])
        msg.pose.position.y = float(T[1, 3])
        msg.pose.position.z = float(T[2, 3])
        qx, qy, qz, qw     = matrix_to_quaternion(T[:3, :3])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    # ── Marker corner geometry ────────────────────────────────────────────────

    def _corners_local(self) -> np.ndarray:
        s = self.marker_size_m / 2.0
        return np.array([[-s, s, 0], [s, s, 0],
                         [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    def _corners_world(self, wp: MarkerWorldPose) -> np.ndarray:
        local = self._corners_local()
        Rz    = yaw_to_matrix(wp.yaw_deg)
        world = (Rz @ local.T).T
        world[:, 0] += wp.x
        world[:, 1] += wp.y
        world[:, 2] += wp.z
        return world.astype(np.float32)

    # ── Angle / distance helpers ──────────────────────────────────────────────

    @staticmethod
    def _angles_from_tvec(tvec: np.ndarray):
        """
        Camera-frame tvec → (dist_m, h_deg, v_deg).
        Camera frame: X=right, Y=down, Z=forward.
        h_deg: positive = marker to the right of bore-sight.
        v_deg: positive = marker above bore-sight.
        """
        tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])
        dist_m     = math.sqrt(tx*tx + ty*ty + tz*tz)
        h_deg      = math.degrees(math.atan2(tx,  tz))
        v_deg      = math.degrees(math.atan2(-ty, tz))
        return dist_m, h_deg, v_deg

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo, drone: str):
        state = self.states[drone]
        if state.camera_info_received:
            return   # only need it once
        state.camera_matrix      = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        state.dist_coeffs        = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        state.camera_info_received = True
        self.get_logger().info(f'[{drone}] camera_info received')

    def _image_cb(self, msg: Image, drone: str):
        state = self.states[drone]
        color = drone_color(drone)

        # ── Fallback intrinsics if camera_info not yet received ───────────────
        if not state.camera_info_received:
            state.camera_matrix = np.array([[140.0, 0.0, 80.0],
                                             [0.0, 140.0, 60.0],
                                             [0.0,   0.0,  1.0]],
                                            dtype=np.float64)
            state.dist_coeffs        = np.zeros((5, 1), dtype=np.float64)
            state.camera_info_received = True
            self.get_logger().warn(
                f'[{drone}] camera_info missing — using fallback intrinsics')

        # ── Decode ───────────────────────────────────────────────────────────
        try:
            if msg.encoding == 'mono8':
                gray  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                gray  = (frame if len(frame.shape) == 2
                         else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        except Exception as e:
            self.get_logger().error(f'[{drone}] cv_bridge: {e}')
            return

        # ── Detect ───────────────────────────────────────────────────────────
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params)

        marker_array = MarkerArray()

        if rejected:
            cv2.aruco.drawDetectedMarkers(frame, rejected,
                                          borderColor=(0, 0, 255))

        if ids is None or len(ids) == 0:
            self.get_logger().info(f'[{drone}] no markers detected')
            self._publish_deleteall(drone, msg.header, marker_array)
            self._publish_debug(drone, frame, msg.header)
            return

        ids_flat = ids.flatten().tolist()
        self.get_logger().info(f'[{drone}] detected IDs: {ids_flat}')

        # ── Per-marker: angles, distances, overlay, RViz cubes ───────────────
        object_points_all = []
        image_points_all  = []
        hud_lines         = []   # collected for the HUD panel

        for idx, marker_id in enumerate(ids_flat):
            c       = corners[idx].reshape(4, 2).astype(np.float32)
            side_px = float(np.linalg.norm(c[0] - c[1]))
            center  = np.mean(c, axis=0).astype(int)
            in_map  = marker_id in self.marker_map

            # ── Single-marker pose for angle/distance readout ─────────────────
            rvec_s, tvec_s, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[idx:idx+1], self.marker_size_m,
                state.camera_matrix, state.dist_coeffs)
            tvec_s = tvec_s[0][0]
            rvec_s = rvec_s[0][0]

            dist_m, h_deg, v_deg = self._angles_from_tvec(tvec_s)

            R_marker, _ = cv2.Rodrigues(rvec_s)
            yaw_deg = math.degrees(math.atan2(R_marker[1, 0], R_marker[0, 0]))

            # ── Log ──────────────────────────────────────────────────────────
            map_tag = "in-map" if in_map else "NOT IN MAP"
            self.get_logger().info(
                f'[{drone}] M{marker_id:2d} [{map_tag}] | '
                f'dist={dist_m:.2f}m | '
                f'H={h_deg:+.1f}deg V={v_deg:+.1f}deg | '
                f'yaw={yaw_deg:+.1f}deg | '
                f'px={side_px:.0f}px')

            # ── Draw detection border — green if in map, red if unknown ───────
            border_color = (0, 255, 0) if in_map else (0, 0, 255)
            cv2.polylines(frame,
                          [corners[idx].reshape((-1, 1, 2)).astype(np.int32)],
                          True, border_color, 2)

            # ── Small label next to detected marker corner ────────────────────
            map_warn = "" if in_map else " !"
            cv2.putText(frame,
                f'M{marker_id}{map_warn}',
                (max(0, int(corners[idx][0][0][0])),
                 max(12, int(corners[idx][0][0][1]) - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, border_color, 1)

            # ── Collect HUD line for this marker ──────────────────────────────
            hud_lines.append(
                (f'M{marker_id:<2d}  {dist_m:.2f}m  '
                 f'H{h_deg:+.1f}  V{v_deg:+.1f}  '
                 f'yaw{yaw_deg:+.1f}',
                 border_color))

            # ── RViz cube + label at world position ───────────────────────────
            self._append_marker_visual(marker_array, msg.header, drone, marker_id)

            # ── Accumulate world-space points for solvePnP ────────────────────
            if in_map:
                object_points_all.append(
                    self._corners_world(self.marker_map[marker_id]))
                image_points_all.append(c)

        self.marker_pubs[drone].publish(marker_array)

        # ── HUD panel — dark background + per-drone header + marker rows ──────
        # Panel sits at top-left. Each row is 13px tall at QQVGA.
        font       = cv2.FONT_HERSHEY_SIMPLEX
        fscale     = 0.32
        fthick     = 1
        row_h      = 13
        pad        = 3
        n_rows     = 1 + len(hud_lines)   # header + one row per marker
        panel_h    = n_rows * row_h + pad * 2
        panel_w    = frame.shape[1]        # full width

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (panel_w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        # Header row — drone name + detected IDs + map-match count
        n_matched = len(object_points_all)
        header_txt = (f'{drone}  |  IDs: {ids_flat}  |  '
                      f'matched: {n_matched}/{len(ids_flat)}')
        cv2.putText(frame, header_txt,
                    (pad, pad + row_h - 3),
                    font, fscale, color, fthick)

        # One row per detected marker
        for row_i, (line_txt, line_color) in enumerate(hud_lines):
            y = pad + (row_i + 2) * row_h - 3
            cv2.putText(frame, line_txt, (pad, y),
                        font, fscale, line_color, fthick)

        # ── solvePnP — global pose from all matched markers ───────────────────
        if not object_points_all:
            self.get_logger().warn(
                f'[{drone}] no markers matched marker_map — no pose published')
            # Still show "NO POSE" on frame
            cv2.putText(frame, 'NO POSE — markers not in map',
                        (pad, panel_h + row_h),
                        font, 0.38, (0, 0, 255), 1)
            self._publish_debug(drone, frame, msg.header)
            return

        obj_pts = np.concatenate(object_points_all, axis=0).reshape(-1, 3)
        img_pts = np.concatenate(image_points_all,  axis=0).reshape(-1, 2)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts,
            state.camera_matrix, state.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE)

        if not ok:
            self.get_logger().warn(f'[{drone}] solvePnP failed')
            cv2.putText(frame, 'solvePnP FAILED',
                        (pad, panel_h + row_h),
                        font, 0.38, (0, 0, 255), 1)
            self._publish_debug(drone, frame, msg.header)
            return

        # solvePnP gives world→camera; invert to get camera in world
        T_cam_world  = self._rt_to_transform(rvec, tvec)
        T_world_cam  = np.linalg.inv(T_cam_world)
        T_world_body = T_world_cam @ self.T_cam_body

        pose_msg = self._transform_to_pose(
            T_world_body, msg.header.stamp, self.world_frame)
        self.pose_pubs[drone].publish(pose_msg)

        # ── Bottom bar — solved world position ────────────────────────────────
        xyz = T_world_body[:3, 3]
        pos_txt = (f'WORLD  x={xyz[0]:.2f}  y={xyz[1]:.2f}  z={xyz[2]:.2f} m')
        bar_y = frame.shape[0] - 4
        cv2.rectangle(frame,
                      (0, frame.shape[0] - row_h - pad),
                      (frame.shape[1], frame.shape[0]),
                      (0, 0, 0), -1)
        cv2.putText(frame, pos_txt, (pad, bar_y),
                    font, fscale, color, fthick)

        # Draw pose axes on the frame
        cv2.drawFrameAxes(
            frame,
            state.camera_matrix, state.dist_coeffs,
            rvec, tvec, self.marker_size_m)

        self._publish_debug(drone, frame, msg.header)

    # ── RViz helpers ─────────────────────────────────────────────────────────

    def _publish_deleteall(self, drone: str, header, marker_array: MarkerArray):
        m          = Marker()
        m.header   = header
        m.ns       = f'{drone}_aruco'
        m.id       = 0
        m.action   = Marker.DELETEALL
        marker_array.markers.append(m)
        self.marker_pubs[drone].publish(marker_array)

    def _append_marker_visual(self, marker_array: MarkerArray,
                               header, drone: str, marker_id: int):
        if marker_id not in self.marker_map:
            return
        wp = self.marker_map[marker_id]
        q  = matrix_to_quaternion(yaw_to_matrix(wp.yaw_deg))

        # Green semi-transparent cube at marker world position
        cube                    = Marker()
        cube.header             = header
        cube.header.frame_id    = self.world_frame
        cube.ns                 = f'{drone}_aruco_known'
        cube.id                 = int(marker_id)
        cube.type               = Marker.CUBE
        cube.action             = Marker.ADD
        cube.pose.position.x    = wp.x
        cube.pose.position.y    = wp.y
        cube.pose.position.z    = wp.z
        cube.pose.orientation.x = q[0]
        cube.pose.orientation.y = q[1]
        cube.pose.orientation.z = q[2]
        cube.pose.orientation.w = q[3]
        cube.scale.x            = self.marker_size_m
        cube.scale.y            = self.marker_size_m
        cube.scale.z            = 0.02
        cube.color              = ColorRGBA(r=0.1, g=0.8, b=0.1, a=0.6)
        marker_array.markers.append(cube)

        # White text label above cube
        text                    = Marker()
        text.header             = header
        text.header.frame_id    = self.world_frame
        text.ns                 = f'{drone}_aruco_text'
        text.id                 = int(1000 + marker_id)
        text.type               = Marker.TEXT_VIEW_FACING
        text.action             = Marker.ADD
        text.pose.position.x    = wp.x
        text.pose.position.y    = wp.y
        text.pose.position.z    = wp.z + 0.25
        text.pose.orientation.w = 1.0
        text.scale.z            = 0.20
        text.color              = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text               = f'ID {marker_id}'
        marker_array.markers.append(text)

    def _publish_debug(self, drone: str, frame: np.ndarray, header):
        if not self.publish_debug or drone not in self.debug_pubs:
            return
        out        = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = header
        self.debug_pubs[drone].publish(out)


# ── Entry point ───────────────────────────────────────────────────────────────

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