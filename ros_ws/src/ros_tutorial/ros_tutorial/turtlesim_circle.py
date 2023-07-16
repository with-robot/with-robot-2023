#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3


class TurtlesimCirclePublisher(Node):
    def __init__(self):
        super().__init__('turtlesim_circle')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # 10: quesize
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear = Vector3(x=2.0, y=.0, z=.0)
        msg.angular = Vector3(x=.0, y=.0, z=1.8)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = TurtlesimCirclePublisher()
    rclpy.spin(publisher)  # blocked until ros2 shutdown

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()