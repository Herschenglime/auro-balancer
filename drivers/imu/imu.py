import smbus2
import time

# I2C setup
I2C_BUS = 1
ISM_ADDR = 0x6B  # Check your i2cdetect output; could be 0x6A too
bus = smbus2.SMBus(I2C_BUS)

# Register addresses
WHO_AM_I      = 0x0F
CTRL1_XL      = 0x10  # Accel config
CTRL2_G       = 0x11  # Gyro config
CTRL3_C       = 0x12  # Common config
OUTX_L_G      = 0x22  # Gyro X low byte
OUTX_L_A      = 0x28  # Accel X low byte

# Helper: read signed 16-bit little endian
def read_sensor_data(base_reg):
    data = bus.read_i2c_block_data(ISM_ADDR, base_reg, 6)
    x = int.from_bytes(data[0:2], 'little', signed=True)
    y = int.from_bytes(data[2:4], 'little', signed=True)
    z = int.from_bytes(data[4:6], 'little', signed=True)
    return (x, y, z)

# --- Init sequence ---
# Confirm chip ID
whoami = bus.read_byte_data(ISM_ADDR, WHO_AM_I)
if whoami != 0x6B:
    print(f"Unexpected WHO_AM_I: 0x{whoami:02X}")
else:
    print("✅ ISM330DHCX detected!")

# Reset the device (CTRL3_C, bit 0)
bus.write_byte_data(ISM_ADDR, CTRL3_C, 0x01)
time.sleep(0.1)

# Enable accelerometer: 104 Hz, ±2g
#bus.write_byte_data(ISM_ADDR, CTRL1_XL, 0x40)  # ODR_XL = 104 Hz, FS_XL = ±2g
# Enable gyro: 104 Hz, 250 dps
bus.write_byte_data(ISM_ADDR, CTRL2_G, 0x40)   # ODR_G = 104 Hz, FS_G = 250 dps

# Main loop
print("Reading sensor data...\nPress Ctrl+C to stop.\n")
try:
    while True:
        #accel = read_sensor_data(OUTX_L_A)
        gyro = read_sensor_data(OUTX_L_G)

        # Convert to physical units (approximate)
        #accel_g = tuple(a / 16384 for a in accel)  # 2g scale → 16384 LSB/g
        gyro_dps = tuple(g / 131 for g in gyro)    # 250 dps → 131 LSB/dps

        #print(f"Accel (g): {accel_g}")
        print(f"Gyro  (°/s): {gyro_dps}")
        print("-" * 40)
        #time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopped by user")
