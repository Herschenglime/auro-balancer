# ros imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3

# communication imports
import smbus2

# python imports
import threading
import time
# import numpy as np

# constants
# I2C setup
I2C_BUS = 1
ISM_ADDR = 0x6B  # Check your i2cdetect output; could be 0x6A too

# Register addresses
WHO_AM_I = 0x0F
CTRL1_XL = 0x10  # Accel config
CTRL2_G = 0x11  # Gyro config
CTRL3_C = 0x12  # Common config
OUTX_L_G = 0x22  # Gyro X low byte
OUTX_L_A = 0x28  # Accel X low byte


class IMUPublisher(Node):
    # Helper: read signed 16-bit little endian
    def read_sensor_data(self, base_reg):
        data = self.bus.read_i2c_block_data(ISM_ADDR, base_reg, 6)
        x = int.from_bytes(data[0:2], "little", signed=True)
        y = int.from_bytes(data[2:4], "little", signed=True)
        z = int.from_bytes(data[4:6], "little", signed=True)
        return (x, y, z)

    def __init__(self):
        super().__init__("imu_publisher")
        # publish an array with the current angular velocity *and* an estimated absolute angle
        # self.publisher_ = self.create_publisher(Float64MultiArray, "dps_deg", 10)

        # use the built-in vector3 type to publish the x, y, and z for both degrees and angle
        self.gyro_publisher = self.create_publisher(Vector3, "gyro_dps", 10)
        self.angle_publisher = self.create_publisher(Vector3, "absolute_angle", 10)

        self.bus = smbus2.SMBus(I2C_BUS)

        # internal absolute angle (in degrees)
        self.abs_ang = (0, 0, 0)

        # --- Init sequence ---
        # Confirm chip ID
        whoami = self.bus.read_byte_data(ISM_ADDR, WHO_AM_I)
        if whoami != 0x6B:
            print(f"Unexpected WHO_AM_I: 0x{whoami:02X}")
        else:
            print("✅ ISM330DHCX detected!")

        # Reset the device (CTRL3_C, bit 0)
        self.bus.write_byte_data(ISM_ADDR, CTRL3_C, 0x01)
        time.sleep(0.1)

        # Enable accelerometer: 104 Hz, ±2g
        # bus.write_byte_data(ISM_ADDR, CTRL1_XL, 0x40)  # ODR_XL = 104 Hz, FS_XL = ±2g
        # Enable gyro: 104 Hz, 250 dps
        self.bus.write_byte_data(
            ISM_ADDR, CTRL2_G, 0x40
        )  # ODR_G = 104 Hz, FS_G = 250 dps

        # setup a thread to continuously poll in the background
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_gyro_loop)
        self.publish_thread.start()

    def publish_gyro_loop(self):
        while self.running:
            # distance = self.vl53.range
            # msg = String()
            # msg.data = f"Distance: {distance} mm"
            # self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg.data)

            # read gyro data

            dt = 0.001

            # accel = read_sensor_data(OUTX_L_A)

            gyro = self.read_sensor_data(OUTX_L_G)

            # Convert to physical units (approximate)
            # accel_g = tuple(a / 16384 for a in accel)  # 2g scale → 16384 LSB/g
            gyro_dps = tuple(g / 131 for g in gyro)  # 250 dps → 131 LSB/dps

            # calcualte current absolute angle based on gyro reading for each axis
            self.abs_ang = tuple(abs + g * dt for abs, g in zip(self.abs_ang, gyro_dps))

            gyro_msg = Vector3()
            gyro_msg.x, gyro_msg.y, gyro_msg.z = gyro_dps

            angle_msg = Vector3()
            angle_msg.x, angle_msg.y, angle_msg.z = self.abs_ang

            self.gyro_publisher.publish(gyro_msg)
            self.angle_publisher.publish(angle_msg)

            gyro_str = ", ".join(f"{g:.2f}" for g in gyro_dps)
            angle_str = ", ".join(f"{a:.2f}" for a in self.abs_ang)

            # self.get_logger().info(
            #     f"Gyro (°/s): ({gyro_str}) | Derived Angle: ({angle_str})"
            # )

            # Sleep a little bit to avoid spamming CPU
            time.sleep(dt)  # 10ms sleep; adjust based on sensor update speed

    def destroy_node(self):
        self.running = False
        self.publish_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
