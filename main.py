import wifi_manager  
from pibot import pibot_client
import time
from colour_printing import print_coloured, bcolors


wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.

bot = pibot_client.PiBot(ip="172.19.232.120")


def main_loop():
    bot.setVelocity(20,20,1)
    print(bot.getEncoders())

try:
    while True:
        main_loop()
        bot.setVelocity(0,0)
        time.sleep(1)

except KeyboardInterrupt:
    wifi_manager.assert_connection_to_network("QUT")