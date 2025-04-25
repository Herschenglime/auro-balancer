import time
import board
import busio
import adafruit_vl53l0x
import numpy as np
i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

start = np.array([])
while True:
    print("Distance: {}mm".format(vl53.range))
