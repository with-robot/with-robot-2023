#!/usr/bin/env python3

import asyncio
from concurrent.futures import ThreadPoolExecutor
import time
import rclpy
from rclpy.node import Node
from .tcp_client import RobotProxy
from sensor_msgs.msg import Image
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSReliabilityPolicy,
)
from .tcp_client import RobotProxy, bridge, cv2, CvBridgeError
import threading
from inf.srv import WheelControlParams


class WheelCameraNode(Node):
    lock = threading.Lock()

    def __init__(self):
        super().__init__("wheel_control_srv")

        self.service = self.create_service(
            srv_type=WheelControlParams,
            srv_name="/wheel_control",
            callback=self.response,
        )

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.publisher = self.create_publisher(
            Image,
            "/camera_image",
            qos_profile=qos_profile,
            # event_callbacks=self.callback,
        )  # 10: quesize

        # 로봇 인스턴스 생성
        self.connect_robot()

        threading.Thread(target=self.publish_camera_img, args=(1,), daemon=True).start()

        self.get_logger().info(f"service server[{self.robot}] starts....")

    def health_checking(self):
        if not self.robot.alive():
            self.timer.cancel()
            self.robot.disconnect()
            self.get_logger().info(f"reconnecting the robot...")
            self.connect_robot()

    def connect_robot(self):
        self.robot = RobotProxy(self.get_logger())
        # 3초마다 체크
        self.timer = self.create_timer(15, self.health_checking)

    def response(self, request, response) -> any:
        cmd = request.cmd
        val = request.val
        self.get_logger().info(f"service req: cmd={cmd}, val={val}")

        response.result = self.robot.send_msg(cmd, val)
        self.get_logger().info(f"service res: {response.result}")
        return response

    def publish_camera_img(self, sleep_: int = 1):
        self.get_logger().info(f"publish_camera_img: {sleep_}")

        while True:
            data: Image = self.robot.capture_image()
            if data:
                self.get_logger().info(f"recv encoding: {data.encoding}")

                try:
                    image = bridge.imgmsg_to_cv2(data, "bgr8")  # "bgr8"

                    (rows, cols, channels) = image.shape
                    self.get_logger().info(
                        f'shape: {str(rows)+":"+str(cols)+":"+str(channels)}'
                    )
                    # cv2.imshow("Sending Image", image)
                    # cv2.waitKey(3)
                    self.publisher.publish(data)

                except CvBridgeError as e:
                    self.get_logger().info(e)

            time.sleep(sleep_)


def main(args=None):
    rclpy.init(args=args)

    wheel_camera = WheelCameraNode()

    rclpy.spin(wheel_camera)  # blocked until ros2 shutdown

    wheel_camera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
