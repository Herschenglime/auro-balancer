import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int16
from geometry_msgs.msg import Vector3

import time
import numpy as np

# constants
SET_POINT_MM = 90  # mm
CONTROL_PERIOD = 0.02

G = 9.81  # m/s^2, used to convert accel

# control gain matrix - calculated for poles with external script
# limit to 20% for now for debugging
# K = 0.25 * np.array([0.76, 1.96, 17.75, 7.0])

KP = 0.4  # proportional: a positive angle moves the ball away from the sensor, so proportinally tilt in the other direction
KD = 0.25  # derivative: account for velocity of ball in control equation


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

        self.accel_sub = self.create_subscription(
            Vector3, "accel_g", self.accel_callback, 10
        )
        self.accel_sub  # prevent unused variable warning

        # state variables
        self.latest_dist = None
        self.latest_gyro_dps = None
        self.dist_prev_err = 0.0  # assume no previous distance error for initial case

        # store previous distance values to smooth with moving average
        self.dist_buffer = []
        self.buf_size = 4  # moving average window size
        self.deadband_mm = 2.0  # if within 2 mm, stop fiddling the motor

        # for control state stuff
        self.latest_accel = None
        self.prev_dist = None
        self.ball_velocity = 0.0
        self.rail_theta = 0.0  # estimated rail angle (radians)
        self.rail_theta_dot = 0.0  # rail angular velocity (rad/s)

        # handle publishing of data
        self.servo_publisher = self.create_publisher(Float64, "servo_angle", 10)

        # set motor to middle position to begin
        self.zero_angle_msg = Float64()
        self.zero_angle_msg.data = 0.0
        self.servo_publisher.publish(self.zero_angle_msg)

        # wait for motor to get to position
        time.sleep(0.5)

        self.servo_angle = 0.0
        # self.up = True
        # self.increment = 0.5

        self.get_logger().info("Control loop starting now.")

        # get last time for more accurate timing measurements
        self.last_time = self.get_clock().now()

        # timer had stale distance data, only update based on when new distance comes in
        # # begin timing loop for pid controller
        # self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

    def dist_callback(self, msg):
        # update buffer
        self.dist_buffer.append(msg.data)
        if len(self.dist_buffer) > self.buf_size:
            self.dist_buffer.pop(0)

        # smooth distance with moving average
        self.latest_dist = sum(self.dist_buffer) / len(self.dist_buffer)

        # rerun control loop when each piece of distance data comes in
        self.control_loop()

    def gyro_callback(self, msg):
        self.latest_gyro_dps = msg

    def accel_callback(self, msg):
        # convert from g to m/s^2 before giving to control code
        self.latest_accel = Vector3(x=msg.x * G, y=msg.y * G, z=msg.z * G)

    def control_loop(self):
        if (
            self.latest_dist is None
            or self.latest_gyro_dps is None
            or self.latest_accel is None
        ):
            # don't do anything until sensors is ready
            return

        # self.get_logger().info(f"latest dist: {self.latest_dist} mm")

        # from observation, z points up, x points perpendicularly out of the contraption and is what gets tilted
        # self.get_logger().info(
        #     f"Gyro (°/s): ({self.latest_gyro_dps.x:.2f}, {self.latest_gyro_dps.y:.2f}, {self.latest_gyro_dps.z:.2f}) | "
        #     f"Accel (m/s²): ({self.latest_accel.x:.2f}, {self.latest_accel.y:.2f}, {self.latest_accel.z:.2f})"
        # )

        # get actual time for more accurate timing calculations
        curr_time = self.get_clock().now()

        # get higher resolution time description
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = CONTROL_PERIOD

        self.last_time = curr_time

        # self.get_logger().info(f"time difference is {dt} s")

        # implement pid code
        dist_err = SET_POINT_MM - self.latest_dist
        # self.get_logger().info(f"err is {dist_err} mm")

        # calculate proportional part
        p = KP * dist_err
        # self.get_logger().info(f"p is {p}")

        # calculate derivative part
        dist_diff = dist_err - self.dist_prev_err

        d = KD * (dist_diff / dt)
        # self.get_logger().info(f"d is {d}")

        # add up final pid calculation
        total = p + d
        # self.get_logger().info(f"total is {total}")

        # clamp total
        if total < -90:
            total = -90.0
        if total > 90:
            total = 90.0

        # deadband small control changes
        if abs(total) < 2.0:  # if motor command < 1 degree, ignore
            total = 0.0

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
