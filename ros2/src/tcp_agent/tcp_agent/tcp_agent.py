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
from .esp32_driver import IOHandler, bridge, cv2, CvBridgeError


class TcpAgent(Node):
    def __init__(self, driver: Any):
        super().__init__("ros_arduino")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.publisher = self.create_publisher(
            Image,
            "/camera_image",
            10,
            # event_callbacks=self.callback,
        )  # 10: quesize

        driver.callback(self.img_receiver)
        driver.connect()

    def img_receiver(self, data: Image):
        self.get_logger().info(f"recv encoding: {data.encoding}")
        try:
            image = bridge.imgmsg_to_cv2(data, "bgr8")

            (rows, cols, channels) = image.shape
            self.get_logger().info(
                f'shape: {str(rows)+":"+str(cols)+":"+str(channels)}'
            )
            cv2.imshow("Sending Image", image)
            cv2.waitKey(3)

            self.publisher.publish(data)

        except CvBridgeError as e:
            self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)

    esp_driver = IOHandler(host="192.168.4.1", port=10000)
    agent = TcpAgent(driver=esp_driver)
    rclpy.spin(agent)  # blocked until ros2 shutdown

    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
