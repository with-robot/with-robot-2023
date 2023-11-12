#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from inf.srv import WheelControlParams
import sys
import tty, termios


class WheelControlClientNode(Node):
    def __init__(self):
        super().__init__("wheel_control_client")

        self.client = self.create_client(
            srv_type=WheelControlParams, srv_name="/wheel_control"
        )

        while not self.client.wait_for_service():
            self.get_logger().info("service not available, waiting again...")
            sleep(2)

    def send_request(self, code, val, theta):
        req = WheelControlParams.Request()
        req.cmd = code
        req.val = val
        req.theta = theta
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=3)

        result = self.future.result()
        # self.res.result = result
        self.get_logger().info(f"service request's result = {result}")
        return result


def getch():
    return input()
    import pty

    pid = pty.fork()
    fd = sys.stdin.fileno()
    if not pid:
        old_settings = termios.tcgetattr(fd)
    else:
        old_settings = None
    try:
        if not pid:
            tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    arrow_up = "i"
    arrow_down = "o"
    arrow_right = "k"
    arrow_left = "j"
    stop = "p"

    # 79:End -> Stop
    key_map = {arrow_up: 1, arrow_down: 5, arrow_left: 2, arrow_right: 4, stop: 3}
    rclpy.init(args=args)

    client_node = WheelControlClientNode()
    while True:
        key = getch()
        if key == "q":
            break

        if key not in key_map:
            continue

        client_node.send_request("direction", key_map.get(key), 1.0)

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
