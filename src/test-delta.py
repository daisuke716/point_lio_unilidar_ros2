#!/usr/bin/env python3
import struct, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

class CloudTimeRange(Node):
    def __init__(self):
        super().__init__("cloud_time_range")
        self.create_subscription(PointCloud2, "/utlidar/cloud", self.cb, qos_profile_sensor_data)
        self.start = time.time()

    def cb(self, msg: PointCloud2):
        step = msg.point_step
        off = next(f.offset for f in msg.fields if f.name == "time")
        count = msg.width * msg.height
        data = msg.data
        times = [struct.unpack_from("<f", data, i * step + off)[0] for i in range(count)]
        tmin, tmax = min(times), max(times)
        print(f"points {count}  time_min {tmin:.6f}  time_max {tmax:.6f}  delta {tmax - tmin:.6f}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CloudTimeRange()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
            if time.time() - node.start > 5:
                print("timeout waiting for cloud", file=sys.stderr)
                break
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
