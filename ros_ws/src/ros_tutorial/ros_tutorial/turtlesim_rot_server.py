#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from turtlesim.action import RotateAbsolute

class TurtlesimRotateServer(Node):
    def __init__(self):
        super().__init__('turtlesim_rot_server')

        self.server = ActionServer(self, RotateAbsolute,
                                   '/turtle1/rotate_absolute',
                                   callback_group=ReentrantCallbackGroup(),
                                   execute_callback=self.execute_callback,
                                   goal_callback=self.goal_callback,
                                   cancel_callback=self.cancel_callback)
        self.get_logger().info('action server started...')
    
   
    def goal_callback(self, goal_request):
        self.get_logger().info(f'recv goal request: {goal_request}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, cancel_request):
        self.get_logger().info(f'recv cancel request: {cancel_request}')
        return CancelResponse.ACCEPT


    async def execute_callback(self, goal_handle):
        feedback = RotateAbsolute.Feedback()
        feedback.remaining = 10.0
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('action canceled...')
                return RotateAbsolute.Result()

            feedback.remaining -= 1
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()
        self.get_logger().info('action succeed...')

        res = RotateAbsolute.Result()
        res.delta = 0.0
        return res


def main(args=None):
    rclpy.init(args=args)

    server = TurtlesimRotateServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)

    server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()