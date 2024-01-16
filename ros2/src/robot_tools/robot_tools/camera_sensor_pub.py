#!/usr/bin/env python3

import time
from typing import Any
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSReliabilityPolicy,
)
from .tcp_client import RobotProxy, bridge, cv2, CvBridgeError


class CmrImageSendPubNode(Node):
    robot_rebooting = False

    def __init__(self):
        super().__init__("camera_img_pub_node")

        # 로봇 인스턴스 생성
        self.connect_robot()
        # 구독자 취득
        self.publisher = self.get_publisher()

        self.create_timer(0.1, self.publish_raw_image)

        self.get_logger().info(f"camera pub[{self.robot}] starts....")

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
            # event_callbacks=self.callback,
        )  # 10: quesize

    def health_checking(self):
        if self.robot.is_alive():
            return

        self.timer.cancel()
        self.robot.disconnect()
        self.get_logger().info(f"reconnecting the robot...")
        self.connect_robot()

    def connect_robot(self):
        self.robot_rebooting = True
        self.robot = RobotProxy(self.get_logger())
        self.robot_rebooting = False
        # 3초마다 체크
        self.timer = self.create_timer(3, self.health_checking)

    def publish_raw_image(self):
        if self.robot_rebooting:
            return
        data: Image = self.robot.capture_image()
        if data:
            # self.get_logger().info(f"recv encoding: {data.encoding}")

            # try:
            # image = bridge.imgmsg_to_cv2(data, "bgr8")  # "bgr8"

            # (rows, cols, channels) = image.shape
            # self.get_logger().info(
            #     f'수신이미지: {str(rows)+":"+str(cols)+":"+str(channels)}'
            # )
            # cv2.imshow("Sending Image", image)
            # cv2.waitKey(3)
            self.publisher.publish(data)

            # except CvBridgeError as e:
            #     self.get_logger().info(e)
        else:
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    pub_node = CmrImageSendPubNode()

    rclpy.spin(pub_node)  # blocked until ros2 shutdown

    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
