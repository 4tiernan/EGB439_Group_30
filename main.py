import wifi_manager  
from pibot.pibot_client import PiBot
from pibot.pibot_sim import PiBotSim
import time
import numpy as np
from colour_printing import print_coloured, bcolors
from navigation import drive_to_segment, navigate, drive_to_line, generate_bernoulli
from pibot.pibot_plot import Bot_Plotter

from pibot.pibot_const import * 

print(max_velocity_command)

use_simulation = True
on_campus = True
if(not use_simulation):
    if(wifi_manager.get_windows_ssid() != "QUT" and wifi_manager.get_windows_ssid() != "EGB439"): # If not connected to QUT or EGB439, try to connect to the penquin pi network.
        on_campus = False

    if(on_campus):
        wifi_manager.assert_connection_to_network("EGB439") # Ensure you are connected to the EGB439 network.
        bot_ip = "172.19.232.120"
        localiser_ip = "egb439localiser1"
        
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
        realtime=True,  # False means run as fast as possible for testing
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

plotter.plt.plot([0,2], [2,0], 'r--', label="Target Line")

# WAYPOINT TRACKING
just_waypoint_tracking = False
# DRIVE TO LINE FLAG
drive_to_line_func = False

# PURE PURSUIT ATTEMPT
bernoulli_path = generate_bernoulli(center=(1.0, 1.0))
print("bernoulli_path", type(bernoulli_path))
path_index = 0
bernoulli_path_ting = True


def bernoulli_segment_follow(current_pose):
    global path_index
    global target_reached
    global desired_heading

    if path_index >= bernoulli_path.shape[1] - 1:
        target_reached = True
        return (0, 0), desired_heading, target_reached

    line_start = bernoulli_path[:, path_index]
    line_end = bernoulli_path[:, path_index + 5]

    velocity_commands, desired_heading, segment_done = drive_to_segment(
        current_pose, line_start, line_end, print_statements=True
    )

    if segment_done:
        path_index += 1

    return velocity_commands, desired_heading, target_reached
  

def update_control():
    global target_pose
    global desired_heading
    global target_reached
    global path_index
    
    # HANDLE PRINTING
    if (bot.realtime == False): # Disabled in case of trying to sim ASAP
        print_statements = False
    else:
        print_statements = True

    current_pose = bot.getLocalizerPose(group_number=30)
    current_pose = (0,0,0) if current_pose is None else current_pose # If localizer fails, assume we are at the origin facing right (0 radians).

    # Just waypoint tracking
    if just_waypoint_tracking:
        velocity_commands, desired_heading = navigate(current_pose, target_pose)

    # 1.2
    if drive_to_line_func:
        velocity_commands, desired_heading, target_reached = drive_to_line(current_pose, print_statements)

    # 1.3
    if bernoulli_path_ting:
        velocity_commands, desired_heading, target_reached = bernoulli_segment_follow(current_pose)

    if not use_simulation and not bot.realtime: # Disabled in case of trying to sim ASAP
        pass # Don't print
    else:
        print(f"Current Pose: {np.round(current_pose, 2)}, Velocity Commands: {np.round(velocity_commands, 2)}")

    bot.move(*velocity_commands)
    #plotter.update(desired_heading=desired_heading)

def simulate(time_now, last_sim_time, last_control_time):

 
    if bot.realtime: #if simulating in realtime
        if time_now - last_control_time >= 0.5: #Update controls at 2hz (mimic localiser speed)
            update_control()
            last_control_time = time_now

        if time_now - last_sim_time >= 0.05: # sim and plot at 20hz for smooth animation
            bot.step(timestep=0.05)
            plotter.update(desired_heading=desired_heading)
            last_sim_time = time_now
    
    # Sim asap if realtime is false        
    else:
        
        update_control()
        bot.step(timestep=0.05)
        plotter.update(desired_heading=desired_heading)

    return last_sim_time, last_control_time

def main_loop():
    
    #Initilise timers
    last_control_time = time.time()
    last_sim_time = time.time()
    
    while True:
        now = time.time()

        # If using sim, do sim stuff
        if use_simulation:
           last_sim_time, last_control_time = simulate(now, last_sim_time, last_control_time)
           if target_reached:
               print("Target Reached")
               print("Closing Sim, stopping program")
               break
         

        # Update control for real robot, no sim or plots required
        else:
            if now - last_control_time >= 0.5: #Run program at 2hz (localiser speed)
                update_control()
                last_control_time = now


def heading_controller():
    while True:
        current_pose = bot.getLocalizerPose(group_number=30)
        angular_correction = -np.deg2rad(current_pose[2])
        bot.move(0,angular_correction, duration=1)
        time.sleep(1)


try:
    main_loop()
    #heading_controller()


except KeyboardInterrupt:
    print("Keyboard Interrupt, stopping robot")
    bot.stop()
    time.sleep(0.5) # Give the stop command time to be sent before exiting.
    if(on_campus and not use_simulation):
        #wifi_manager.assert_connection_to_network("QUT")
        pass