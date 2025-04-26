from gpiozero import Device, Servo
from time import sleep

# set "pin factory" to pigpio to reduce jitters
from gpiozero.pins.pigpio import PiGPIOFactory


Device.pin_factory = PiGPIOFactory()
# https://randomnerdtutorials.com/raspberry-pi-pwm-python/

# using gpio 18 which is on pin 12, but the lib bases it on gpio num
servo = Servo(pin=18)

val = 0
up = True
increment = 0.01
sleep_time = 0.01

# do a servo sweep
try:
    print("initializing...")
    while True:
        servo.min()
        sleep(0.5)
        servo.mid()
        sleep(0.5)
        servo.max()
        sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")
