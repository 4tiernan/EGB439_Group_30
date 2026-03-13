import wifi_manager  
from pibot.pibot_client import PiBot
from pibot.pibot_sim import PiBotSim
import time
import numpy as np
from colour_printing import print_coloured, bcolors
import nav
from pibot.pibot_plot import Bot_Plotter

from pibot.pibot_const import * 

use_simulation = False
on_campus = True
realtime = True # Set to false to run as fast as possible (good for testing), true to run in real time

if(not use_simulation):
    if(wifi_manager.get_windows_ssid() != "QUT" and wifi_manager.get_windows_ssid() != "EGB439"): # If not connected to QUT or EGB439, try to connect to the penquin pi network.
        on_campus = False

    if(on_campus):
        wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.
        bot_ip = "172.19.232.120"
        localiser_ip = "egb439localiser2"
        
    else:
        wifi_manager.assert_connection_to_network("penguinpi:07:c5:ca")
        bot_ip = "10.42.0.1"
        localiser_ip = None

    bot = PiBot(ip=bot_ip, localiser_ip=localiser_ip)

else: # Using Simulator
    print_coloured("Running in SIMULATION mode. No real robot will be controlled.", bcolors.WARNING)
    bot = PiBotSim(
        pose=np.array([1.0, 1.0, 0.8]),
        dt=0.05,
        realtime=realtime,  # False means run as fast as possible for testing
    )

plotter = Bot_Plotter(bot)
desired_heading = None
target_reached = False
bot.stop()
bot.resetEncoder()
time.sleep(1)
#print(bot.getEncoders())
#print(f"Voltage: {bot.getVoltage()}")
start = time.time()

available_controllers = ["waypoint", "drive_to_line", "bernoulli_path"]
selected_controller = available_controllers[1] # Change this index to select different controllers (0, 1, or 2)
print(f"Selected Controller: {selected_controller}")
target_path = nav.initalise_controller(selected_controller)
plotter.plot_target_path(target_path)

def stop_program():
    bot.stop()
    plotter.keep_plot()
    if(on_campus and not use_simulation):
        wifi_manager.assert_connection_to_network("QUT")
        pass
    exit()
    

def update_control():
    global desired_heading
    global target_reached
    
    print_statements = False if use_simulation and bot.realtime == False else True # Disabled in case of trying to sim ASAP, otherwise enabled for real robot or realtime sim

    current_pose = bot.getLocalizerPose(group_number=30)
    current_pose = (0,0,0) if current_pose is None else current_pose # If localizer fails, assume we are at the origin facing right (0 radians).

    velocity_commands, desired_heading, target_reached = nav.run_controller(current_pose, target_path, selected_controller)
    bot.move(*velocity_commands)

    if(print_statements):
        print(f"Current Pose: {np.round(current_pose, 2)}, Velocity Commands: {np.round(velocity_commands, 2)}")

    bot.move(*velocity_commands)
    
    if(use_simulation): # If simulating and not in realtime, step the sim after each control update to see the effect of each command.
        bot.step(timestep=0.1)
        plotter.update(pose=bot.pose, desired_heading=desired_heading) # Use sim internal pose for smooth plotting

    else:
        plotter.update(pose=current_pose, desired_heading=desired_heading)

    if(target_reached):
        print("Target Reached, Stopping Robot")
        stop_program()
        


def main_loop():
    #Initilise timers
    last_control_time = time.time()
    
    while True:
        now = time.time()
        # If realtime, run update at 10Hz
        if (use_simulation and bot.realtime == False) or now - last_control_time >= 0.1: 
            update_control()
            last_control_time = now

try:
    main_loop()

except KeyboardInterrupt:
    print("Keyboard Interrupt, stopping robot")
    stop_program()