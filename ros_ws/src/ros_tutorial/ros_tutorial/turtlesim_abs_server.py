#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute


class TurtlesimAbsoluteServer(Node):
    def __init__(self):
        super().__init__('turtlesim_abs_server')

        self.client = self.create_service(TeleportAbsolute,
                                          '/turtle1/teleport_absolute',
                                          self.service_callback)
    
    def service_callback(self, request, response):
        print('request:', request)
        print('response:', response)
        return response


def main(args=None):
    rclpy.init(args=args)

    service = TurtlesimAbsoluteServer()
    rclpy.spin(service)

    service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()