#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.count = 0;
        self.subscriber = self.create_subscription(
                Int64, "number", self.callback_number, 10)
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.server = self.create_service(SetBool, "reset_counter",
                self.callback_reset_counter)
        self.get_logger().info("Number counter has started")

    def callback_number(self, number_msg):
        self.count += number_msg.data;
        count_msg = Int64()
        count_msg.data = self.count
        self.publisher.publish(count_msg)
        self.get_logger().info("Publishing count " + str(self.count))

    def callback_reset_counter(self, request, response):
        if request.data:
            self.count = 0
            response.message = "Reset count to zero"
        else:
            response.message = "No change"
        response.success = True
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

