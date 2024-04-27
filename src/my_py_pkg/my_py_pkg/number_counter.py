#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.count = 0;
        self.subscriber = self.create_subscription(
                Int64, "number", self.callback_number, 10)
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("Number counter has started")

    def callback_number(self, number_msg):
        self.count += number_msg.data;
        count_msg = Int64()
        count_msg.data = self.count
        self.publisher.publish(count_msg)
        self.get_logger().info("Publishing count " + str(self.count))

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

