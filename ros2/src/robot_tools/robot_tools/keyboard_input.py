import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")
        self.publisher = self.create_publisher(String, "/keyboard_input", 10)
        self.timer_period = 0.1  # 0.1초마다 키보드 입력 확인
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        key_event = keyboard.read_event()
        if key_event.event_type == keyboard.KEY_DOWN:
            if key_event.name == "up":
                msg = String()
                msg.data = "forward"
                self.publisher.publish(msg)
            elif key_event.name == "down":
                msg = String()
                msg.data = "backward"
                self.publisher.publish(msg)
            elif key_event.name == "left":
                msg = String()
                msg.data = "left"
                self.publisher.publish(msg)
            elif key_event.name == "right":
                msg = String()
                msg.data = "right"
                self.publisher.publish(msg)

            self.get_logger().info(f"key={msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
