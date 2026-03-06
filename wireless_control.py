import cv2
import time
import argparse
from pibot.pibot_client import PiBot   # change if your file name is different


FORWARD_SPEED = 80
TURN_SPEED = 60


def main():
    bot = PiBot(ip="172.19.232.120")

    print("Arrow keys to drive")
    print("UP    = forward")
    print("DOWN  = backward")
    print("LEFT  = turn left")
    print("RIGHT = turn right")
    print("Q     = quit")

    while True:
        key = cv2.waitKey(50)

        # Arrow keys
        if key == 82:  # up
            bot.setVelocity(FORWARD_SPEED, FORWARD_SPEED)

        elif key == 84:  # down
            bot.setVelocity(-FORWARD_SPEED, -FORWARD_SPEED)

        elif key == 81:  # left
            bot.setVelocity(-TURN_SPEED, TURN_SPEED)

        elif key == 83:  # right
            bot.setVelocity(TURN_SPEED, -TURN_SPEED)

        elif key == ord('q'):
            break

        else:
            bot.stop()

    bot.stop()


if __name__ == "__main__":
    main()