#!/usr/bin/env python3
# ultra_frame_capture.py — 单帧抓取 /ultrasonic, 打印与 `ros2 topic echo` 同款字段格式 (2026-10-05)
# 用途: ultra_failclosed_check.sh 的帧采集; 走 rclpy 直接订阅 —— 不依赖 ros2 CLI/daemon
#   (lyrical 上 `ros2 topic echo` 与 daemon 有 XMLRPC 兼容问题: unknown tag TopicEndpointInfo)。
# 用法: source install/setup.bash && python3 ultra_frame_capture.py; 10s 无帧退出码 2。
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mechdog_ultrasonic.msg import UltrasonicArray


class Grab(Node):
    def __init__(self):
        super().__init__("ultra_frame_capture")
        self.got = None
        self.sub = self.create_subscription(
            UltrasonicArray, "/ultrasonic", self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        self.got = msg


def main():
    rclpy.init()
    node = Grab()
    t0 = time.time()
    while rclpy.ok() and node.got is None and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    msg = node.got
    node.destroy_node()
    rclpy.shutdown()
    if msg is None:
        print("NO_FRAME (10s 内未收到 /ultrasonic)")
        return 2

    def b2s(v):
        return "true" if v else "false"

    print("front_left_cm: %r" % msg.front_left_cm)
    print("front_center_cm: %r" % msg.front_center_cm)
    print("front_right_cm: %r" % msg.front_right_cm)
    print("bottom_cm: %r" % msg.bottom_cm)
    print("front_left_valid: %s" % b2s(msg.front_left_valid))
    print("front_center_valid: %s" % b2s(msg.front_center_valid))
    print("front_right_valid: %s" % b2s(msg.front_right_valid))
    print("bottom_valid: %s" % b2s(msg.bottom_valid))
    print("period_sec: %r" % msg.period_sec)
    print("seq: %r" % msg.seq)
    return 0


if __name__ == "__main__":
    sys.exit(main())
