#!/usr/bin/env python3
"""Publish the TFs  <ns>/odom -> <ns>/base_link  (and optionally  map -> <ns>/odom)  from PX4.

PX4 (uXRCE-DDS) publishes /<ns>/fmu/out/vehicle_odometry in NED (or FRD) / FRD, ROS uses
ENU / FLU (REP-103), so the pose is converted:
  world: NED (x north, y east, z down) -> ENU (x east, y north, z up)
         FRD (x fwd,   y right, z down) -> FLU (x fwd, y left, z up)
  body : FRD -> FLU
The odom frame is per vehicle: PX4 odometry origin is the position of each vehicle at startup.
Frames are prefixed with the namespace so several vehicles can share /tf.

Common map frame (optional, parameters map_lat / map_lon / map_alt): `map` is a local ENU frame
anchored at that geodetic origin (the scene's `origin`). The EKF origin of each vehicle
(vehicle_local_position ref_lat / ref_lon / ref_alt, i.e. where odom = 0) is converted to ENU
relative to the map origin and published as a static TF map -> <ns>/odom. The rotation is the
identity because the odometry is already ENU (north aligned); this holds for pose_frame NED.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# World frame changes (rows: new axes expressed in the old frame).
_NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])
_BODY_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def quat_to_matrix(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    """Rotation matrix -> (w, x, y, z)."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s,
                (m[1, 2] + m[2, 1]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
            0.25 * s)


def px4_pose_to_ros(position, q_wxyz, pose_frame):
    """PX4 pose (world NED/FRD, body FRD) -> ROS pose (world ENU/FLU, body FLU).

    Returns (position[3], quaternion (x, y, z, w)) or None if the frame is unknown.
    """
    if pose_frame == VehicleOdometry.POSE_FRAME_NED:
        r_world = _NED_TO_ENU
    elif pose_frame == VehicleOdometry.POSE_FRAME_FRD:
        r_world = _FRD_TO_FLU
    else:
        return None
    pos = r_world @ np.asarray(position, dtype=float)
    # R_ros = R_world * R_px4 * R_body^-1  (R_body is its own inverse)
    rot = r_world @ quat_to_matrix(*q_wxyz) @ _BODY_FRD_TO_FLU
    w, x, y, z = matrix_to_quat(rot)
    return pos, (x, y, z, w)


# WGS84
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(lat_deg, lon_deg, alt):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    return np.array([
        (n + alt) * math.cos(lat) * math.cos(lon),
        (n + alt) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - _WGS84_E2) + alt) * math.sin(lat),
    ])


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Geodetic point (deg, deg, m AMSL) -> local ENU (east, north, up) relative to the origin."""
    d = geodetic_to_ecef(lat, lon, alt) - geodetic_to_ecef(lat0, lon0, alt0)
    sl, cl = math.sin(math.radians(lat0)), math.cos(math.radians(lat0))
    so, co = math.sin(math.radians(lon0)), math.cos(math.radians(lon0))
    return np.array([
        -so * d[0] + co * d[1],
        -sl * co * d[0] - sl * so * d[1] + cl * d[2],
        cl * co * d[0] + cl * so * d[1] + sl * d[2],
    ])


class OdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        ns = self.get_namespace().strip('/')
        prefix = f'{ns}/' if ns else ''
        self.declare_parameter('odom_frame', f'{prefix}odom')
        self.declare_parameter('base_frame', f'{prefix}base_link')
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._warned_frame = False

        # Optional common map frame (all three origin parameters must be set).
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_lat', float('nan'))
        self.declare_parameter('map_lon', float('nan'))
        self.declare_parameter('map_alt', float('nan'))
        self._map_frame = self.get_parameter('map_frame').value
        self._map_origin = (self.get_parameter('map_lat').value,
                            self.get_parameter('map_lon').value,
                            self.get_parameter('map_alt').value)
        self._map_enabled = all(math.isfinite(v) for v in self._map_origin)
        self._last_ref = None

        self._broadcaster = TransformBroadcaster(self)
        # PX4 publishes best-effort sensor-like data.
        self.create_subscription(
            VehicleOdometry, 'fmu/out/vehicle_odometry', self._on_odometry,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'TF {self._odom_frame} -> {self._base_frame} from '
            f'{self.get_namespace()}/fmu/out/vehicle_odometry')

        if self._map_enabled:
            self._static_broadcaster = StaticTransformBroadcaster(self)
            # PX4 versioned messages are published as <topic>_v<MESSAGE_VERSION>
            version = VehicleLocalPosition.MESSAGE_VERSION
            topic = 'fmu/out/vehicle_local_position' + (f'_v{version}' if version > 0 else '')
            self.create_subscription(
                VehicleLocalPosition, topic, self._on_local_position, qos_profile_sensor_data)
            self.get_logger().info(
                f'TF {self._map_frame} -> {self._odom_frame} from the EKF origin of '
                f'{self.get_namespace()}/{topic} (map origin lat/lon/alt = {self._map_origin})')

    def _on_local_position(self, msg):
        if not (msg.xy_global and msg.z_global):
            return  # EKF origin not set yet
        ref = (msg.ref_lat, msg.ref_lon, float(msg.ref_alt))
        if not all(math.isfinite(v) for v in ref) or ref == self._last_ref:
            return
        self._last_ref = ref
        e, n, u = geodetic_to_enu(*ref, *self._map_origin)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = float(e)
        t.transform.translation.y = float(n)
        t.transform.translation.z = float(u)
        t.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(t)  # latched: late subscribers get it
        self.get_logger().info(
            f'{self._map_frame} -> {self._odom_frame}: E={e:.3f} N={n:.3f} U={u:.3f} m '
            f'(EKF origin {ref[0]:.7f}, {ref[1]:.7f}, {ref[2]:.2f})')

    def _on_odometry(self, msg):
        if not (np.all(np.isfinite(msg.position)) and np.all(np.isfinite(msg.q))):
            return  # PX4 uses NaN for unknown position/attitude
        if (self._map_enabled and msg.pose_frame == VehicleOdometry.POSE_FRAME_FRD
                and not self._warned_frame):
            self.get_logger().warn(
                'pose_frame is FRD: map -> odom assumes a north-aligned (NED) odometry, '
                'the heading offset is not applied')
            self._warned_frame = True
        result = px4_pose_to_ros(msg.position, msg.q, msg.pose_frame)
        if result is None:
            if not self._warned_frame:
                self.get_logger().warn(f'unknown pose_frame {msg.pose_frame}, no TF published')
                self._warned_frame = True
            return
        pos, (qx, qy, qz, qw) = result

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # sim time with use_sim_time
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self._broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
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
