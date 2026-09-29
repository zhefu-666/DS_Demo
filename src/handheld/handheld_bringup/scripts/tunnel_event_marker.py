#!/usr/bin/env python3
"""阶段 3 人工事件打点：放入/移走箱体时按键，向 /tunnel_event 发布事件名。

tunnel_status_node 收到事件后开始计时，下一次稳定状态切换时输出延迟并写入 CSV。
事件话题也会被录进 bag，方便回放复现。

按键：p = box_placed（放入箱体）  r = box_removed（移走箱体）  q = 退出
"""

import sys
import termios
import tty

import rclpy
from std_msgs.msg import String

KEYS = {"p": "box_placed", "r": "box_removed"}


def read_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("tunnel_event_marker")
    pub = node.create_publisher(String, "/tunnel_event", 10)
    print("p=放入箱体  r=移走箱体  q=退出")
    try:
        while rclpy.ok():
            key = read_key().lower()
            if key in ("q", "\x03"):
                break
            if key in KEYS:
                pub.publish(String(data=KEYS[key]))
                node.get_logger().info(f"事件: {KEYS[key]}")
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
