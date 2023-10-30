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


class CarmeraImgSubscriber(Node):
    def __init__(self):
        super().__init__("ros_arduino")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.subscriber = self.create_subscription(
            Image, "/camera_image", self.topic_callback, qos_profile
        )  # 10: quesize
        self.bridge = CvBridge()
        self.seq = itertools.count()

    def topic_callback(self, message: Image):
        self.get_logger().info(f"recv msg incoding: {message.encoding}")
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")

            (rows, cols, channels) = image.shape
            self.get_logger().info(
                f'shape: {str(rows)+":"+str(cols)+":"+str(channels)}'
            )

            cv2.imshow("Receiving Image", image)
            cv2.waitKey(3)

            sn = next(self.seq)
            filename = f"camera_{sn:03d}.jpg"
            self.get_logger().info(f"파일명: {filename}")

            # cv2.imwrite(filename, image)

        except CvBridgeError as e:
            self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)

    subscriber = CarmeraImgSubscriber()
    rclpy.spin(subscriber)  # blocked until ros2 shutdown

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
