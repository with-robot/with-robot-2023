#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .tcp_client import RobotProxy
from inf.srv import WheelControlParams


class WheelControlSrvNode(Node):
    def __init__(self):
        super().__init__("wheel_control_srv")

        self.service = self.create_service(
            srv_type=WheelControlParams,
            srv_name="/wheel_control",
            callback=self.response,
        )

        # 로봇 인스턴스 생성
        self.robot = RobotProxy(self.get_logger())

        self.get_logger().info("service server starts....")

    def response(self, request, response) -> any:
        cmd = request.cmd
        val = request.val
        self.get_logger().info(f"cmd={cmd}, val={val}")

        response.result = self.robot.send_msg(cmd, val)
        self.get_logger().info(f"service return = {response.result}")
        return response


def main(args=None):
    rclpy.init(args=args)

    server_node = WheelControlSrvNode()
    rclpy.spin(server_node)  # blocked until ros2 shutdown

    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
