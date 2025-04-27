from gpiozero import Device, Servo
from time import sleep

# set "pin factory" to pigpio to reduce jitters
from gpiozero.pins.pigpio import PiGPIOFactory


Device.pin_factory = PiGPIOFactory()
# https://randomnerdtutorials.com/raspberry-pi-pwm-python/

# using gpio 18 which is on pin 12, but the lib bases it on gpio num
# default pulse width limits on library is too small, use the default ones from the arduino lib
servo = Servo(18, min_pulse_width=0.544 / 1000, max_pulse_width=2.4 / 1000)

val = 0
up = True
increment = 0.05
sleep_time = 0.05

# do a servo sweep
try:
    # print("initializing...")
    # servo.max()
    # sleep(0.5)
    servo.mid()
    sleep(0.5)
    # servo.min()
    # sleep(0.5)
    while True:
        servo.value = val

        if up:
            val += increment
            if val >= 1:
                val = 1
                up = False
        else:
            val -= increment
            if val <= -1:
                val = -1
                up = True
        # print("val is ", val)
        sleep(sleep_time)

except KeyboardInterrupt:
    print("Program stopped")
