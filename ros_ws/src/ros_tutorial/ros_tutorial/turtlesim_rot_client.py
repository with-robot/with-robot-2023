#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute


class TurtlesimRotateClient(Node):
    def __init__(self):
        super().__init__('turtlesim_rot_client')

        self.client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')
        self.get_logger().info('action client started...')
    
    def send_goal(self, theta):
        goal_req = RotateAbsolute.Goal()
        goal_req.theta = theta

        if self.client.wait_for_server(10) is False:
            self.get_logger().info('service not available...')
            return
        
        goal_future = self.client.send_goal_async(goal_req, feedback_callback=self.feedback_callback)
        goal_future.add_done_callback(self.goal_callback)
    
    def goal_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('gole rejected...')
            return

        self.get_logger().info('gole accepted...')
        goal_future = self.goal_handle.get_result_async()
        goal_future.add_done_callback(self.result_callback)
        # cancel test
        # self.timer= self.create_timer(1.0, self.timer_callback)

    def feedback_callback(self, msg):
        feedback = msg.feedback
        self.get_logger().info(f'recv feedback: {feedback.remaining}')

    def result_callback(self, future):
        result_handle = future.result()
        res = result_handle.result
        self.get_logger().info(f'recv result: {res.delta}')

        self.destroy_node()
        rclpy.shutdown()
    
    def timer_callback(self):
        self.get_logger().info(f'canceling goal...')

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_callback)
        # stop timer
        self.timer.cancel()
    
    def cancel_callback(self, future):
        result_handle = future.result()
        if len(result_handle.goals_canceling) > 0:
            self.get_logger().info('canceling goal success...')
        else:
            self.get_logger().info('canceling goal fail...')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    client = TurtlesimRotateClient()
    client.send_goal(3.14)

    rclpy.spin(client)


if __name__ == '__main__':
    main()