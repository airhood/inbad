import RPi.GPIO as GPIO
import threading
import time
from robot import Robot

GPIO.setmode(GPIO.BOARD)

robot = Robot(1, (15, 11, 13), (19, 21, 23), '/dev/ttyUSB0', 'broker_ip', 1883, 1)
robot.start()

robot_thread = threading.Thread(target=robot.start, daemon=True)
robot_thread.start()

print("robot running...")

while True:
    time.sleep(1)
