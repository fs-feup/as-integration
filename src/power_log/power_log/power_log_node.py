#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import psutil
from std_msgs.msg import Float32, Float32MultiArray, Int64MultiArray


def pc_stats():
    temps = psutil.sensors_temperatures()
    cpu_temp = None
    if "k10temp" in temps:
        for entry in temps["k10temp"]:
            if entry.label == "Tctl":
                cpu_temp = entry.current
                break

    return {
        "cpu_core_usage": psutil.cpu_percent(interval=0.1, percpu=True),
        "memory_usage": psutil.virtual_memory().percent,
        "cpu_temperature": cpu_temp,
    }


class PowerNode(Node):
    def __init__(self):
        super().__init__("power_log_node")
        self.cpu_publisher = self.create_publisher(
            Float32MultiArray, "cpu_core_usage", 10
        )
        self.memory_publisher = self.create_publisher(Float32, "memory_usage", 10)
        self.temperature_publisher = self.create_publisher(
            Float32, "cpu_temperature", 10
        )
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        stats = pc_stats()

        cpu_core_usage_msg = Float32MultiArray()
        cpu_core_usage_msg.data = stats["cpu_core_usage"]
        self.cpu_publisher.publish(cpu_core_usage_msg)
        
        memory_msg = Float32()
        memory_msg.data = stats["memory_usage"]
        self.memory_publisher.publish(memory_msg)

        temperature_msg = Float32()
        temperature_msg.data = stats["cpu_temperature"]
        self.temperature_publisher.publish(temperature_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
