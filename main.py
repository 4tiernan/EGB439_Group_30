import wifi_manager  
from pibot import pibot_client
import time

wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.

bot = pibot_client.PiBot(ip="172.19.232.120")


def main_loop():
    time.sleep(1)

try:
    while True:
        main_loop()
except KeyboardInterrupt:
    wifi_manager.assert_connection_to_network("QUT")