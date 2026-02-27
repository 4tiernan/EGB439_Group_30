from pibot_client import PiBot
import time

bot = PiBot(ip="172.19.232.120")


bitmask = 0b0011110111010011


'''for i in range(2):
    bot.setLEDArray(bitmask)

    time.sleep(1)

    bot.setLEDArray(0)

    time.sleep(1)'''


'''for x in range(3):
    for i in range(2**16):

        bot.setLEDArray(i)

        time.sleep(0.005)
'''

i = 0
for k in range(4):
    i = 1 << k
    i = i | (1 << (k * 4))
    bot.setLEDArray(i)

    time.sleep(0.5)



print(bot.getLocalizerPose(30))

for i in range(10):
    bot.setLED(2, False)
    bot.setLED(3, True)

    time.sleep(1)

    bot.setLED(2, True)
    bot.setLED(3, False)

    time.sleep(1)