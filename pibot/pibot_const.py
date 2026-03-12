import numpy as np

maximum_pwm_command = 255
maximum_linear_velocity = 0.3 # m/s
maximum_angular_velocity = 2 # radians/s

pid_velocity_factor = 10 # ratio between set velocity and encoder clicks per second
encoder_clicks_per_rotation = 390
wheel_diameter = 0.065 # m
wheel_spacing = 0.17 # m

wheel_circumference = wheel_diameter * np.pi
encoder_clicks_per_m = encoder_clicks_per_rotation / wheel_circumference
encoder_clicks_rad = wheel_spacing / 2 * encoder_clicks_per_m

max_velocity_command = (maximum_linear_velocity * encoder_clicks_per_m) / pid_velocity_factor