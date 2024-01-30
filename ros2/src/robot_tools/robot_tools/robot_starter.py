#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from .websocket_client import RobotProxy, bridge, cv2, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSReliabilityPolicy,
)
import threading
import asyncio
import nest_asyncio

nest_asyncio.apply()


class WheelCameraNode(Node):
    robot_rebooting = False

    def __init__(self):
        super().__init__("websocket_node")

        # 로봇 인스턴스 생성
        self.connect_robot()

        # 이미지센서 발행자 생성
        self.publisher = self.get_publisher()

        # 키보드입력 구독자 생성
        self.subscriber = self.get_subscription()

    def start_publish(self):
        self.create_timer(0.1, callback=self._publish_image_raw)
        # _t = threading.Thread(
        #     target=self._publish_image_raw,
        #     args=(
        #         loop,
        #         0.1,
        #     ),
        #     daemon=True,
        # )
        # _t.start()
        self.get_logger().info(f"service server[{self.robot}] starts....")

    def get_publisher(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        return self.create_publisher(
            Image,
            "/image_raw",
            qos_profile=qos_profile,
        )  # 10: quesize

    def get_subscription(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )
        return self.create_subscription(Twist, "/cmd_vel", self._handler, qos_profile)

    def health_checking(self):
        if not self.robot.is_alive():
            self.timer.cancel()
            self.robot.disconnect()
            self.get_logger().info(f"reconnecting the robot...")
            self.connect_robot()

    def connect_robot(self):
        self.robot_rebooting = True
        self.robot = RobotProxy(self.get_logger())
        self.robot_rebooting = False
        # 3초마다 체크
        # self.timer = self.create_timer(15, self.health_checking)

    def _handler(self, data: Twist) -> any:
        if self.robot_rebooting:
            time.sleep(0.1)
            return
        # 8bit -
        x = data.linear.x
        # y = data.linear.y
        # z = data.linear.z
        # ax = data.angular.x
        # ay = data.angular.y
        az = data.angular.z
        # key_map = {arrow_up: 1, arrow_down: 5, arrow_left: 2, arrow_right: 4, stop: 3}
        # 전진, x=0.5, 후진, x=-05, 좌회전: az=0.8, 우회전: az=0.8
        self.get_logger().info(f"linear={x}, angular={az}")

        result = asyncio.create_task(
            self.robot.send_msg("direction", self.cal_val(x, az))
        )
        if not result:
            self.connect_robot()

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

    def _publish_image_raw(self, sleep_: int = 0.1):
        loop = asyncio.get_event_loop()
        # asyncio.set_event_loop(loop)

        # while True:
        data: Image = loop.run_until_complete(self.robot.capture_image())

        if data:
            # self.get_logger().info(f"recv encoding: {data.encoding}")

            try:
                # image = bridge.imgmsg_to_cv2(data, "bgr8")  # "bgr8"

                # (rows, cols, channels) = image.shape
                # self.get_logger().info(
                #     f'shape: {str(rows)+":"+str(cols)+":"+str(channels)}'
                # )
                # cv2.imshow("Sending Image", image)
                # cv2.waitKey(3)

                self.publisher.publish(data)

            except CvBridgeError as e:
                self.get_logger().info(e)
                # break
                #
        # time.sleep(sleep_)


async def spinning(node: WheelCameraNode):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop: asyncio):
    rclpy.init(args=args)

    wheel_camera = WheelCameraNode()
    wheel_camera.start_publish()

    spin_task = loop.create_task(spinning(wheel_camera))
    # connect_task = loop.create_task(wheel_camera.start_publish())

    # await asyncio.gather(spin_task, connect_task)
    await spin_task

    wheel_camera.destroy_node()
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == "__main__":
    main()
