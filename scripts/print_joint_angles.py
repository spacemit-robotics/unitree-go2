#!/usr/bin/env python3
"""用中文打印 Go2 的 12 个关节侧位置角度。"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_NAMES_ZH = {
    "FR_hip_joint": "前右腿髋关节",
    "FR_thigh_joint": "前右腿大腿关节",
    "FR_calf_joint": "前右腿小腿关节",
    "FL_hip_joint": "前左腿髋关节",
    "FL_thigh_joint": "前左腿大腿关节",
    "FL_calf_joint": "前左腿小腿关节",
    "RR_hip_joint": "后右腿髋关节",
    "RR_thigh_joint": "后右腿大腿关节",
    "RR_calf_joint": "后右腿小腿关节",
    "RL_hip_joint": "后左腿髋关节",
    "RL_thigh_joint": "后左腿大腿关节",
    "RL_calf_joint": "后左腿小腿关节",
}

DISPLAY_ORDER = (
    ("FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"),
    ("FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"),
    ("RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"),
    ("RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"),
)


class JointAnglePrinter(Node):
    def __init__(self) -> None:
        super().__init__("print_joint_angles")
        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("interval_sec", 0.5)

        topic = self.get_parameter("topic").value
        self.interval_sec = float(self.get_parameter("interval_sec").value)
        if not math.isfinite(self.interval_sec) or self.interval_sec < 0.0:
            raise ValueError("interval_sec 必须是大于或等于 0 的有限数值")

        self.last_print_time = -math.inf
        self.subscription = self.create_subscription(
            JointState,
            topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"正在监听 {topic}，每 {self.interval_sec:g} 秒打印一次关节侧角度"
        )

    def joint_state_callback(self, message: JointState) -> None:
        now = time.monotonic()
        if now - self.last_print_time < self.interval_sec:
            return

        positions = dict(zip(message.name, message.position))
        missing = [name for leg in DISPLAY_ORDER for name in leg if name not in positions]
        if missing:
            self.get_logger().warning(
                "关节状态不完整，缺少：" + ", ".join(missing)
            )
            return

        self.last_print_time = now
        print("\n========== 12 个电机关节侧位置（角度制） ==========", flush=False)
        for leg in DISPLAY_ORDER:
            values = [
                f"{JOINT_NAMES_ZH[name]}：{math.degrees(positions[name]):8.2f}°"
                for name in leg
            ]
            print(" | ".join(values), flush=False)
        print("====================================================", flush=True)


def main() -> None:
    rclpy.init()
    node = JointAnglePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
