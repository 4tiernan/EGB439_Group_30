import wifi_manager  
from pibot.pibot_client import PiBot
from pibot.pibot_sim import PiBotSim
import time
import numpy as np
from colour_printing import print_coloured, bcolors
from navigation import navigate

use_simulation = True
on_campus = True
if(not use_simulation):
    if(wifi_manager.get_windows_ssid() != "QUT" and wifi_manager.get_windows_ssid() != "EGB439"): # If not connected to QUT or EGB439, try to connect to the penquin pi network.
        on_campus = False

    if(on_campus):
        wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.
        bot_ip = "172.19.232.120"
        localiser_ip = "172.19.232.104"
        
    else:
        wifi_manager.assert_connection_to_network("penguinpi:07:c5:ca")
        bot_ip = "10.42.0.1"
        localiser_ip = None

    bot = PiBot(ip=bot_ip, localiser_ip=localiser_ip)

else: # Using Simulator
    print_coloured("Running in SIMULATION mode. No real robot will be controlled.", bcolors.WARNING)
    bot = PiBotSim(
        pose=np.array([0.5, 0.5, 0.0]),
        dt=0.05,
        realtime=False,  # True means run as fast as possible for testing)
    )

bot.resetEncoder()
#time.sleep(1)
#print(bot.getEncoders())
#print(f"Voltage: {bot.getVoltage()}")


def update_control():
    current_pose = bot.getLocalizerPose(group_number=30)
    current_pose = (0,0,0) if current_pose is None else current_pose # If localizer fails, assume we are at the origin facing right (0 radians).

    target_pose = (1, 1, 0) # Example target pose (x, y, theta)
    velocity_commands = navigate(current_pose, target_pose)
    print(f"Current Pose: {current_pose}, Target Pose: {target_pose}, Velocity Commands: {velocity_commands}")
    bot.move(*velocity_commands)

def main_loop():
    last_loop_time = time.time()
    last_sim_time = time.time()
    while True:
        if(time.time() - last_loop_time >= 0.5): # Run the main loop at 2 Hz
            update_control()
            last_loop_time = time.time()
        
        if(use_simulation and time.time() - last_sim_time >= 0.05): # Update the simulation at 20 Hz for smooth animation
            bot.update_simulation()
            last_sim_time = time.time()

try:
    main_loop()

except KeyboardInterrupt:
    if(use_simulation):
        bot.stop_simulation()
    else: 
        bot.stop()
    if(on_campus and not use_simulation):
        wifi_manager.assert_connection_to_network("QUT")