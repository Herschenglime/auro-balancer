# ros imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

# gpio imports
import board
import busio
import adafruit_vl53l0x

# python imports
import threading
import time
# import numpy as np


class DistPublisher(Node):
    def __init__(self):
        super().__init__("dist_publisher")
        self.publisher_ = self.create_publisher(Int16, "dist_mm", 10)

        # setup distance sensor device
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = adafruit_vl53l0x.VL53L0X(self.i2c)

        # start = np.array([])

        # setup a thread to continuously poll in the background
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_distance_loop)
        self.publish_thread.start()

    def publish_distance_loop(self):
        while self.running:
            distance = self.vl53.range
            msg = Int16()
            msg.data = distance
            self.publisher_.publish(msg)
            self.get_logger().info(f"Distance: {distance} mm")

            # Sleep a little bit to avoid spamming CPU
            time.sleep(0.001)  # 10ms sleep; adjust based on sensor update speed

    def destroy_node(self):
        self.running = False
        self.publish_thread.join()
        super().destroy_node()


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
