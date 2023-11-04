#!/usr/bin/env python3

from typing import Any
import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
from .tcp_client import RobotProxy
from turtlesim.srv import TeleportAbsolute


class WheelControlSrvNode(Node):
    def __init__(self):
        super().__init__("wheel_control_srv")

        self.service = self.create_service(
            srv_type=TeleportAbsolute, srv_name="/wheel_control", callback=self.response
        )

        # 로봇 인스턴스 생성
        self.robot = RobotProxy(self.get_logger())

    def response(
        self, request: TeleportAbsolute.Request, response: TeleportAbsolute.Response
    ):
        print("request:", request)
        print("response:", response)
        r = self.robot.send_msg(request.x)
        return r


def main(args=None):
    rclpy.init(args=args)

    server_node = WheelControlSrvNode()
    rclpy.spin(server_node)  # blocked until ros2 shutdown

    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
