#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from functools import partial
import time

from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client_node")
        self.callback_group = ReentrantCallbackGroup()
        self.call_count = 0

        # Timer fires every 0.1 seconds, make 20 service calls total
        self.timer = self.create_timer(0.1, self.make_service_call, callback_group=self.callback_group)

    def make_service_call(self):
        if self.call_count < 20:
            a, b = self.call_count, self.call_count + 1
            self.call_count += 1
            self.call_add_two_ints_server(a, b)
        else:
            self.timer.cancel()

    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server add_two_ints_server")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        # future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

        # Use custom wait function instead of native future handling
        if self._wait_till_future_complete(future):
            self.callback_call_add_two_ints(future, a, b)
        else:
            self.get_logger().error(f"Future timed out for {a} + {b}")
        client.destroy()


    def _wait_till_future_complete(self, fut: rclpy.Future,
                                timeout: float = 3.0):
        """
        Waits till futures gets completed or timeout whichever is earlier
        """
        stime, counts = timeout / 5.0, 0
        while rclpy.ok() and counts <= 5:
            if not fut.done():
                self.get_logger().info(f"Future not done sleeping for {stime} {counts}")
                time.sleep(stime)
                counts = counts + 1
            else:
                return True
        fut.cancel()
        return False

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " + str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

