import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Int16


# take in sensor data, perform filtering and PID controls, then set to cotnroller
class KalmanPID(Node):
    def __init__(self):
        super().__init__("kalman_pid")

        # subscribe to distnace and gyro readings
        self.subscription = self.create_subscription(
            Int16, "dist_mm", self.dist_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.latest_dist = None
        self.latest_gyro_dps = None

    def dist_callback(self, msg):
        self.latest_dist = msg.data


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
