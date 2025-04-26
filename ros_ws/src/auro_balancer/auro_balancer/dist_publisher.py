# ros imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# gpio imports
import board
import busio
import adafruit_vl53l0x
# import numpy as np


class DistPublisher(Node):
    def __init__(self):
        super().__init__("dist_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # setup distance sensor device
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = adafruit_vl53l0x.VL53L0X(self.i2c)

        # start = np.array([])

    def timer_callback(self):
        msg = String()
        msg.data = "Distance: %dmm" % self.vl53.range
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    dist_publisher = DistPublisher()

    rclpy.spin(dist_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
