import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from gpiozero import Device, Servo, AngularServo
from time import sleep

# set "pin factory" to pigpio to reduce jitters
from gpiozero.pins.pigpio import PiGPIOFactory


Device.pin_factory = PiGPIOFactory()
# https://randomnerdtutorials.com/raspberry-pi-pwm-python/


# subscribe to filter node - from that, get where to set the motor to
class ServoController(Node):
    def __init__(self):
        super().__init__("servo_controller")
        self.subscription = self.create_subscription(
            Float64, "servo_angle", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # using gpio 18 which is on pin 12, but the lib bases it on gpio num
        # default pulse width limits on library is too small, use the default ones from the arduino lib
        self.servo = AngularServo(
            18, min_pulse_width=0.544 / 1000, max_pulse_width=2.4 / 1000
        )

    def listener_callback(self, msg):
        self.servo.angle = msg.data

        self.get_logger().info(f"Setting servo to angle: {msg.data} °")


def main(args=None):
    rclpy.init(args=args)

    servo_controller = ServoController()

    rclpy.spin(servo_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
