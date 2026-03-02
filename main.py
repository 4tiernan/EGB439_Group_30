import wifi_manager  
from pibot import pibot_client
import time
from colour_printing import print_coloured, bcolors

on_campus = True
if(wifi_manager.get_windows_ssid() != "QUT" and wifi_manager.get_windows_ssid() != "EGB439"): # If not connected to QUT or EGB439, try to connect to the penquin pi network.
    on_campus = False

if(on_campus):
    wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.
    bot = pibot_client.PiBot(ip="172.19.232.120")
else:
    wifi_manager.assert_connection_to_network("penguinpi:07:c5:ca")
    bot = pibot_client.PiBot(ip="10.42.0.1")

bot.resetEncoder()
bot.setVelocity(255,255)
time.sleep(1)
bot.stop()
print(bot.getEncoders())
print(f"Voltage: {bot.getVoltage()}")

def main_loop():
    a=1


try:
    while True:
        main_loop()
        bot.setVelocity(0,0)
        time.sleep(1)

except KeyboardInterrupt:
    if(on_campus):
        wifi_manager.assert_connection_to_network("QUT")