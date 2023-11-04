#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute


class WheelControlClientNode(Node):
    def __init__(self):
        super().__init__("wheel_control_client")

        self.client = self.create_client(
            srv_type=TeleportAbsolute, srv_name="/wheel_control"
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = TeleportAbsolute.Request()

    def send_request(self, x, y, theta):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        result = self.future.result()
        self.get_logger().info(f"result = {result}")
        return result


def main(args=None):
    rclpy.init(args=args)

    client_node = WheelControlClientNode()
    client_node.send_request(10.0, 10.0, 1.0)

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
