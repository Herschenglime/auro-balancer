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
K = np.array([45.9, 22.6, 78.7, 20.5])


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

        # begin timing loop for pid controller
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

    def dist_callback(self, msg):
        self.latest_dist = msg.data

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

        self.get_logger().info(f"latest dist: {self.latest_dist} mm")

        # from observation, z points up, x points perpendicularly out of the contraption and is what gets tilted
        self.get_logger().info(
            f"Gyro (°/s): ({self.latest_gyro_dps.x:.2f}, {self.latest_gyro_dps.y:.2f}, {self.latest_gyro_dps.z:.2f}) | "
            f"Accel (m/s²): ({self.latest_accel.x:.2f}, {self.latest_accel.y:.2f}, {self.latest_accel.z:.2f})"
        )

        # get actual time for more accurate timing calculations
        curr_time = self.get_clock().now()

        # get higher resolution time description
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = CONTROL_PERIOD

        self.last_time = curr_time

        self.get_logger().info(f"time difference is {dt} s")

        ### implement state space control code ###
        # ball pos relative to center
        ball_pos = self.latest_dist - SET_POINT_MM

        # estimate velocity w/ position difference (assuming we have previous measured already)
        if self.prev_dist is not None:
            self.ball_velocity = (self.latest_dist - self.prev_dist) / dt  # in mm/s

        # record last dist for next time
        self.prev_dist = self.latest_dist

        # use accelerometer to estimate current rail tilt
        # z is vertically up, y is to right along rail, and x faces towards user
        accel_x = self.latest_accel.x
        accel_z = self.latest_accel.y
        # y shouldn't change, so we ignore

        # get angle of rail in radians based on acceleration
        self.rail_theta = np.arctan2(accel_x, accel_z)

        # also estimate angular velocity - rail rotates along x axis
        # (convert to radians for easier physics)
        self.rail_theta_dot = np.deg2rad(self.latest_gyro_dps.x)

        # create state vector (a la kalman stuff)
        x_vec = np.array(
            [
                ball_pos / 1000.0,  # convert mm to meters
                self.ball_velocity / 1000.0,  # convert mm/s to m/s
                self.rail_theta,
                self.rail_theta_dot,
            ]
        )

        # control law (thanks Quintin!) u = -Kx
        u = -np.dot(K, x_vec)  # K is also a vector of control params, so this is scalar

        # clamp servo angle to be within it's operating range (that we set in servo controller)
        u_deg = np.clip(np.rad2deg(u), -90.0, 90.0)

        # publish to servo
        servo_msg = Float64()
        servo_msg.data = u_deg
        self.servo_publisher.publish(servo_msg)

        # log measured quantities
        self.get_logger().info(
            f"Ball pos: {ball_pos:.2f}mm, Ball vel: {self.ball_velocity:.2f}mm/s, "
            f"Rail θ: {np.degrees(self.rail_theta):.2f}°, Rail θ̇: {np.degrees(self.rail_theta_dot):.2f}°/s, "
            f"Servo: {u_deg:.2f}°"
        )


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
