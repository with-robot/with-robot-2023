#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState


class FakeDriver(Node):
    def __init__(self):
        super().__init__('fake_driver')

        # joint states publisher
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', 10)
        # cmd vel subscriber
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # vel raw publisher
        self.pub_vel_raw = self.create_publisher(TwistStamped, 'vel_raw', 10)

        # init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = 'joint_states'
        self.joint_states.name = ['left_wheel_joint', 'right_wheel_joint']
        self.joint_states.position = [0.0, 0.0]

        self.vel_raw = TwistStamped()

        # timer
        self.timer = self.create_timer(0.1, self.publish_callback)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'recv cmd_vel message {msg}')
        self.vel_raw.twist = msg
    
    def publish_callback(self):
        curr_time = self.get_clock().now()

        # joint states
        self.joint_states.header.stamp = curr_time.to_msg()

        # vel raw
        self.vel_raw.header.stamp = curr_time.to_msg()

        # publish
        self.pub_joint_states.publish(self.joint_states)
        self.pub_vel_raw.publish(self.vel_raw)

        # simulate wheel rotate
        self.joint_states.position[0] += 0.05
        self.joint_states.position[1] += 0.05


def main(args=None):
    rclpy.init(args=args)

    driver = FakeDriver()
    executor = MultiThreadedExecutor()
    rclpy.spin(driver, executor=executor)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()