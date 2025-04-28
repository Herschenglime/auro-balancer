import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3

import time

# constants
SET_POINT = 95  # mm
KP = 0.5  # proportional: a positive angle moves the ball away from the sensor, so proportinally tilt in the other direction


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
        self.zero_angle_msg = Float64()
        self.zero_angle_msg.data = 0.0
        self.servo_publisher.publish(self.zero_angle_msg)

        # wait for motor to get to position
        time.sleep(0.5)

        self.servo_angle = 0.0
        # self.up = True
        # self.increment = 0.5

        self.get_logger().info("Control loop starting now.")

        # begin timing loop for pid controller
        self.control_timer = self.create_timer(0.01, self.control_loop)

    def dist_callback(self, msg):
        self.latest_dist = msg.data

    def gyro_callback(self, msg):
        self.latest_gyro_dps = msg

    def control_loop(self):
        if self.latest_dist is None:
            # don't do anything until distance is ready
            return

        # implement pid code
        dist_err = SET_POINT - self.latest_dist

        # calculate proportional part
        p = KP * dist_err

        # add up final pid calculation
        total = p

        if total < -90:
            total = -90.0
        if total > 90:
            total = 90.0

        # capture previous error
        self.dist_prev_err = dist_err

        # publish to servo
        servo_msg = Float64()
        servo_msg.data = total

        self.servo_publisher.publish(servo_msg)


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
