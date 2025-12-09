"""Quick helper node to compute IMU/LiDAR timestamp difference on /utlidar topics."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class StampDiff(Node):
    def __init__(self) -> None:
        super().__init__("stamp_diff_once")
        self.lidar_stamp = None
        self.imu_stamp = None
        self.start = time.time()
        self.create_subscription(
            PointCloud2, "/utlidar/cloud", self.lidar_cb, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/utlidar/imu", self.imu_cb, qos_profile_sensor_data)

    def lidar_cb(self, msg) -> None:
        self.lidar_stamp = msg.header.stamp
        self._check_and_exit()

    def imu_cb(self, msg) -> None:
        self.imu_stamp = msg.header.stamp
        self._check_and_exit()

    def _check_and_exit(self) -> None:
        if self.lidar_stamp and self.imu_stamp:
            tl = self.lidar_stamp.sec + self.lidar_stamp.nanosec * 1e-9
            ti = self.imu_stamp.sec + self.imu_stamp.nanosec * 1e-9
            self.get_logger().info(
                f"lidar: {tl:.9f}  imu: {ti:.9f}  imu-lidar: {ti - tl:.6f} s"
            )
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = StampDiff()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            if time.time() - node.start > 5:
                print("timeout waiting for both topics", file=sys.stderr)
                break
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
