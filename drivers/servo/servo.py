from gpiozero import Device, Servo
from time import sleep

# set "pin factory" to pigpio to reduce jitters
from gpiozero.pins.pigpio import PiGPIOFactory


Device.pin_factory = PiGPIOFactory()
# https://randomnerdtutorials.com/raspberry-pi-pwm-python/

# using gpio 18 which is on pin 12, but the lib bases it on gpio num
servo = Servo(18)

val = -1

# do a servo sweep
try:
    servo.mid()
except KeyboardInterrupt:
    print("Program stopped")
