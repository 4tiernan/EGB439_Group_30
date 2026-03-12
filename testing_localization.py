import argparse
import time
import math as m
import cv2
import numpy as np
from pibot.pibot_client import PiBot
import wifi_manager  


def angle_difference(angle1, angle2):
    # Calculate the difference and normalize it to [-180, 180]
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff


target_angle = -170
acrtual_angle = 180

angle_change = angle_difference(target_angle, acrtual_angle)
print(angle_change)
print(target_angle - acrtual_angle)