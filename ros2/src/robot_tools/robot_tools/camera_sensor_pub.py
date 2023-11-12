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
        self.robot = RobotProxy(self.get_logger())

    def subscribe(self):
        # 구독자로 등록한다.
        self.robot.subscribe(self)
        self.robot.req_capture()

    def desubscribe(self, s):
        # 구독을 중지한다.
        self.robot.desubscribe(s)

    def update(self, data: Image):
        self.get_logger().info(f"recv encoding: {data.encoding}")
        try:
            image = bridge.imgmsg_to_cv2(data, "bgr8")

            (rows, cols, channels) = image.shape
            self.get_logger().info(
                f'shape: {str(rows)+":"+str(cols)+":"+str(channels)}'
            )
            # cv2.imshow("Sending Image", image)
            # cv2.waitKey(3)

            self.publisher.publish(data)

        except CvBridgeError as e:
            self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)

    pub_node = CmrImageSendPubNode()
    pub_node.subscribe()

    rclpy.spin(pub_node)  # blocked until ros2 shutdown

    pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
