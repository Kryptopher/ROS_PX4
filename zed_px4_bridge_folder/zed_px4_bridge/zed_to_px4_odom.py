import math
from typing import List, Sequence, Tuple

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]

ENU_TO_NED: Matrix3 = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)

FRD_TO_FLU: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def quat_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> Matrix3:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quat_wxyz(m: Matrix3) -> List[float]:
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s

    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return [float(w / norm), float(x / norm), float(y / norm), float(z / norm)]


def enu_to_ned_vector(x: float, y: float, z: float) -> List[float]:
    return [float(y), float(x), float(-z)]


def flu_to_frd_body_vector(x: float, y: float, z: float) -> List[float]:
    return [float(x), float(-y), float(-z)]


def covariance_variance(covariance: Sequence[float], indexes: Tuple[int, int, int]) -> List[float]:
    return [float(max(covariance[index], 1e-6)) for index in indexes]


class ZedToPx4Odometry(Node):
    def __init__(self) -> None:
        super().__init__('zed_to_px4_odometry')

        self.declare_parameter('odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('px4_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('quality', 100)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        px4_topic = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.pub = self.create_publisher(VehicleOdometry, px4_topic, px4_qos)

        self.get_logger().info(f'Bridging ZED odometry {odom_topic} to PX4 {px4_topic}')

    def odom_callback(self, msg: Odometry) -> None:
        px4_msg = VehicleOdometry()

        now_us = self.get_clock().now().nanoseconds // 1000
        px4_msg.timestamp = now_us
        px4_msg.timestamp_sample = now_us
        px4_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        px4_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        position = msg.pose.pose.position
        px4_msg.position = enu_to_ned_vector(position.x, position.y, position.z)

        orientation = msg.pose.pose.orientation
        ros_rotation = quat_xyzw_to_matrix(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        px4_rotation = matmul(matmul(ENU_TO_NED, ros_rotation), FRD_TO_FLU)
        px4_msg.q = matrix_to_quat_wxyz(px4_rotation)

        linear = msg.twist.twist.linear
        px4_msg.velocity = enu_to_ned_vector(linear.x, linear.y, linear.z)

        angular = msg.twist.twist.angular
        px4_msg.angular_velocity = flu_to_frd_body_vector(angular.x, angular.y, angular.z)

        px4_msg.position_variance = covariance_variance(msg.pose.covariance, (0, 7, 14))
        px4_msg.orientation_variance = covariance_variance(msg.pose.covariance, (21, 28, 35))
        px4_msg.velocity_variance = covariance_variance(msg.twist.covariance, (0, 7, 14))
        px4_msg.reset_counter = 0
        px4_msg.quality = max(-1, min(100, int(self.quality)))

        self.pub.publish(px4_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedToPx4Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
