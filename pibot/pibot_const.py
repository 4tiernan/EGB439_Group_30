import numpy as np

maximum_pwm_command = 255
maximum_linear_velocity = 0.2 # m/s
maximum_angular_velocity = 0.1 # radians/s

encoder_clicks_per_rotation = 200
wheel_diameter = 0.065 # m
wheel_spacing = 0.147 # m

wheel_circumference = wheel_diameter * np.pi
encoder_clicks_per_m = encoder_clicks_per_rotation / wheel_circumference
encoder_clicks_rad = wheel_spacing / 2 * encoder_clicks_per_m
