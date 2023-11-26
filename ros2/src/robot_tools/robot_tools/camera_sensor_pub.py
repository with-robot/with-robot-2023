#!/usr/bin/env python3

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
    host: str = "192.168.4.1"
    port: int = 10000

    def __init__(self):
        super().__init__("camera_pub_robot_node")

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

        self.create_timer(1, self.publish_camera_img)

        self.get_logger().info(f"camera pub[{self.robot}] starts....")

    def health_checking(self):
        if not self.robot.is_alive():
            self.timer.cancel()
            self.robot.disconnect()
            self.get_logger().info(f"reconnecting the robot...")
            self.connect_robot()

    def connect_robot(self):
        self.robot = RobotProxy(self.get_logger())
        # 3초마다 체크
        self.timer = self.create_timer(3, self.health_checking)

    def publish_camera_img(self):
        data: Image = self.robot.capture_image()
        if data:
            self.get_logger().info(f"recv encoding: {data.encoding}")

            try:
                image = bridge.imgmsg_to_cv2(data, "bgr8")  # "bgr8"

                (rows, cols, channels) = image.shape
                self.get_logger().info(
                    f'수신이미지: {str(rows)+":"+str(cols)+":"+str(channels)}'
                )
                # cv2.imshow("Sending Image", image)
                # cv2.waitKey(3)
                self.publisher.publish(data)

            except CvBridgeError as e:
                self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)

    pub_node = CmrImageSendPubNode()

    rclpy.spin(pub_node)  # blocked until ros2 shutdown

    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
