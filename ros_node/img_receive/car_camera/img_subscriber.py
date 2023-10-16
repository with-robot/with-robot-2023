#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy
import cv2
from cv_bridge import CvBridge

class CarmeraImgSubscriber(Node):
    def __init__(self):
        super().__init__('ros_arduino')

        qos_profile = QoSProfile(            
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        self.subscriber = self.create_subscription(Image, '/camera_image',
                                                   self.topic_callback, qos_profile)  # 10: quesize
        self.bridge = CvBridge()

    def topic_callback(self, msg):
        self.get_logger().info(f'recv mes: {msg.data}')
        image = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite('camera.jpg', image)
        raise Exception()


def main(args=None):
    rclpy.init(args=args)

    subscriber = CarmeraImgSubscriber()
    rclpy.spin(subscriber)  # blocked until ros2 shutdown

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
