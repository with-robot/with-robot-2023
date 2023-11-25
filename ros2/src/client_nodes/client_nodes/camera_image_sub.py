#!/usr/bin/env python3

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
import cv2
from cv_bridge import CvBridge, CvBridgeError
import itertools


class CarmeraImgSubscribeNode(Node):
    def __init__(self):
        super().__init__("camera_sub_client_node")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.subscriber = self.create_subscription(
            Image, "/camera_image", self.image_handler, qos_profile
        )  # 10: quesize
        self.bridge = CvBridge()
        self.seq = itertools.count()

    def image_handler(self, camera_image_data: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(camera_image_data, "bgr8")

            # 이후 처리 로직
            (rows, cols, channels) = image.shape
            self.get_logger().info(
                f'[image_handler] camera_image_shape: {str(rows)+":"+str(cols)+":"+str(channels)}'
            )

            # cv2.imshow("Camera Image", image)
            # cv2.waitKey(1)

            sn = next(self.seq)
            filename = f"camera_{sn:03d}.jpg"
            self.get_logger().info(f"파일명: {filename}")

            cv2.imwrite(filename, image)

        except CvBridgeError as e:
            self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)

    subscribe_node = CarmeraImgSubscribeNode()
    rclpy.spin(subscribe_node)  # blocked until ros2 shutdown

    subscribe_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
