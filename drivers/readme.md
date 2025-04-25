A package was installed and I eft the relics here so you can see what package.
Additionally: 
sudo apt install libgpiod2 python3-libgpiod
sudo apt install python3-rpi.gpio
pip install adafruit-circuitpython-vl53l0x --break-system-packages
pip install smbus2 --break-system-packages

last two are most important and may need to be run on the version of python you
get for ROS


/imu has imu script (simple)
/distance_sensor has the distance sensor
