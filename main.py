import wifi_manager  
from pibot.pibot_client import PiBot
import time
from colour_printing import print_coloured, bcolors
from navigation import navigate

on_campus = True
if(wifi_manager.get_windows_ssid() != "QUT" and wifi_manager.get_windows_ssid() != "EGB439"): # If not connected to QUT or EGB439, try to connect to the penquin pi network.
    on_campus = False

if(on_campus):
    wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.
    bot = PiBot(ip="172.19.232.120")
else:
    wifi_manager.assert_connection_to_network("penguinpi:07:c5:ca")
    bot = PiBot(ip="10.42.0.1")

bot.resetEncoder()
#time.sleep(1)
#print(bot.getEncoders())
#print(f"Voltage: {bot.getVoltage()}")


def main_loop():
    bot.move(0.1, duration=1)
    current_pose = bot.getLocalizerPose(group_number=30)
    target_pose = (1, 1, 0) # Example target pose (x, y, theta)
    velocity_commands = navigate(current_pose, target_pose)


try:
    while True:
        main_loop()
        time.sleep(0.1)

except KeyboardInterrupt:
    bot.stop()
    if(on_campus):
        wifi_manager.assert_connection_to_network("QUT")