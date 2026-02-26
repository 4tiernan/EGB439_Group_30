import argparse
import time
import cv2
from pibot_client import PiBot


# Change the IP address of the following line to match the IP address of your 
# robot. If you have issues, check which network you are on (Need to be on 
# the EGB439 network, not QUT). Also check that you are using the correct conda
# environment.
bot = PiBot(ip="10.42.0.1")

print(f'Voltage: {bot.getVoltage():.2f}V')
print(f'Current: {bot.getCurrent():.2f}A')

enc_begin_left, enc_begin_right = bot.getEncoders()
print(f"get encoders state at beginning: {enc_begin_left}, {enc_begin_right}")

print("test left motor")
bot.setVelocity(20,0)
time.sleep(1)
bot.setVelocity(-20,0)
time.sleep(1)

print("test right motor")
bot.setVelocity(0,20)
time.sleep(1)
bot.setVelocity(0,-20)
time.sleep(1)

print("stop")
bot.setVelocity(0,0)

enc_end_left, enc_end_right = bot.getEncoders()
print(f"get encoders state at end: {enc_end_left}, {enc_end_right}")


print("initialise camera")
time.sleep(2)
print("grab image")
image = bot.getImage()
print(f"image size {image.shape[0]} by {image.shape[1]}")

try:
    while True:
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        image = bot.getImage()
except KeyboardInterrupt:
    exit()
