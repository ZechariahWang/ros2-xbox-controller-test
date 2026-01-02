import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')

        # Subscribe to Xbox controller
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # allow Arduino reset

        self.last_state = 0

    def joy_callback(self, msg):
        a_button = msg.buttons[0]  # A button

        # Press → ON
        if a_button == 1 and self.last_state == 0:
            self.ser.write(b'1')
            self.get_logger().info("LED ON")

        # Release → OFF
        elif a_button == 0 and self.last_state == 1:
            self.ser.write(b'0')
            self.get_logger().info("LED OFF")

        self.last_state = a_button


def main():
    rclpy.init()
    node = LedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
