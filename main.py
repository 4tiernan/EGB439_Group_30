from pibot_client import *


bot = PiBot(ip="172.19.232.120")



# testing wheels for 3 seconds
# bot.setVelocity(right_motor, left_motor, duration=if needed)
# anticlockwise is negative, clockwise is position, currently using PWM signals.
# bot.setVelocity(-50, -50, duration=3)
print("Wheels set to 50 for 3 seconds")
bot.setVelocity(100, 100, 7, 3)
bot.stop()

