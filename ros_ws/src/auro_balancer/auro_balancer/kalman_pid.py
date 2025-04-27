import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3

import time


# take in sensor data, perform filtering and PID controls, then set to cotnroller
class KalmanPID(Node):
    def __init__(self):
        super().__init__("kalman_pid")

        self.get_logger().info("Initializing components...")

        # subscribe to distnace and gyro readings
        self.dist_sub = self.create_subscription(
            Int16, "dist_mm", self.dist_callback, 10
        )
        self.dist_sub  # prevent unused variable warning

        self.gyro_sub = self.create_subscription(
            Vector3, "gyro_dps", self.gyro_callback, 10
        )
        self.gyro_sub  # prevent unused variable warning

        self.latest_dist = None
        self.latest_gyro_dps = None

        # handle publishing of data
        self.servo_publisher = self.create_publisher(Float64, "servo_angle", 10)

        # set motor to middle position and reset gyro to "zero out" measured angle
        zero_angle_msg = Float64()
        zero_angle_msg.data = 0.0
        self.servo_publisher.publish(zero_angle_msg)

        # wait for motor to get to position
        time.sleep(0.5)

        # temporary variables for servo sweep
        self.servo_angle = 0.0
        self.up = True
        self.increment = 0.5

        self.get_logger().info("Control loop starting now.")

        # begin timing loop for pid controller
        self.control_timer = self.create_timer(0.01, self.control_loop)

    def dist_callback(self, msg):
        self.latest_dist = msg.data

    def gyro_callback(self, msg):
        self.latest_gyro_dps = msg

    def control_loop(self):
        # test servo sweep
        servo_msg = Float64()
        servo_msg.data = self.servo_angle

        self.servo_publisher.publish(servo_msg)

        if self.up:
            self.servo_angle += self.increment
            if self.servo_angle >= 90:
                self.servo_angle = 90.0
                self.up = False
        else:
            self.servo_angle -= self.increment
            if self.servo_angle <= -90:
                self.servo_angle = -90.0
                self.up = True


def main(args=None):
    rclpy.init(args=args)

    kalman_pid = KalmanPID()

    rclpy.spin(kalman_pid)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kalman_pid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
