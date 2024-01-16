#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .tcp_client import RobotProxy
from geometry_msgs.msg import Twist
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSReliabilityPolicy,
)
import time


class WheelCmdSubNode(Node):
    def __init__(self):
        super().__init__("wheel_cmd_sub")
        self.is_reboot = False

        # 로봇 인스턴스 생성
        self.connect_robot()
        # 구독자 생성
        self.subscriber = self.get_subscription()

        self.get_logger().info(f"cmd subscription[{self.robot}] starts....")

    def get_subscription(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )
        return self.create_subscription(Twist, "/cmd_vel", self._handler, qos_profile)

    def connect_robot(self):
        self.robot = RobotProxy(self.get_logger())
        # 3초마다 체크
        self.timer = self.create_timer(3, self.health_checking)

    def health_checking(self):
        if not self.robot.is_alive():
            self.timer.cancel()
            self.robot.disconnect()
            self.get_logger().info(f"reconnecting the robot...")
            self.connect_robot()

    def _handler(self, data: Twist) -> any:
        if self.is_reboot:
            time.sleep(0.1)
            return
        # 8bit -
        x = data.linear.x
        y = data.linear.y
        z = data.linear.z

        ax = data.angular.x
        ay = data.angular.y
        az = data.angular.z
        # key_map = {arrow_up: 1, arrow_down: 5, arrow_left: 2, arrow_right: 4, stop: 3}
        # 전진, x=0.5, 후진, x=-05, 좌회전: az=0.8, 우회전: az=0.8
        self.get_logger().info(f"linear={x}, angular={az}")

        result = self.robot.send_msg("direction", self.cal_val(x, az))
        if not result:
            self.is_reboot = True
            # self.subscriber.destroy()
            self.connect_robot()
            # self.subscriber = self.get_subscription()
            self.is_reboot = False

    def cal_val(self, x: float, Z: float) -> bytes:
        # cmd #83  val: - 4비트X 0~15:4비트Z ; 중심 7
        # 직진좌회전  71 x:0.5, z: 1.0       0001 0001
        # 직진        72  x:0.5             0001 0000     i
        # 직진우회전  73 x:0.5, z: -1.0      0001 1001
        # 좌회전      74    z:1.0           0000 0001
        # 우회전      75    z:-1.0          0000 1001
        # 정지        76                    0000 0000
        # 좌측후진    77    x:-0.5, z:-1.0  1001 1001      m
        # 후진        78     x:-0.5         1001 0000      ,
        # 우측후진    79   x:-0.5, z:1.0    1001 0001
        # 회전 #83
        # cmd #84  val - 라인속도 10%
        # cmd #85  val - 각속도 10%

        # cmd #85  val - 최대속도 10%
        # x값을 4비트로 변환한다.
        self.get_logger().info(f"x={x}, Z={Z}")

        l_sign = "0" if x >= 0 else "1"
        x = min(7, int(abs(x) / 0.49))

        a_sign = "0" if Z >= 0 else "1"
        Z = min(7, int(abs(Z / 0.95)))
        # 4비트 첫자리는 모터/각도 방향
        return int(
            l_sign + format(x, "03b") + a_sign + format(Z, "03b"),
            2,
        )

    # .to_bytes(1, byteorder="little")


def main(args=None):
    rclpy.init(args=args)

    server_node = WheelCmdSubNode()
    rclpy.spin(server_node)  # blocked until ros2 shutdown

    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # main()
    server_node = WheelCmdSubNode()
