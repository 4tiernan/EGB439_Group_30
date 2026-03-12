import numpy as np
def angle_diff(target, current):
    return (target - current + np.pi) % (2*np.pi) - np.pi

def navigate(from_pose, to_pose):
    # Simple heading controller
    dist_to_target = np.sqrt((to_pose[0] - from_pose[0])**2 + (to_pose[1] - from_pose[1])**2)
    desired_heading = np.arctan2( to_pose[1] - from_pose[1], to_pose[0] - from_pose[0])
    heading_error = angle_diff(desired_heading, from_pose[2])

    print(f"Current Heading:{round(np.rad2deg(from_pose[2]))}")
    print(f"Target Heading:{round(np.rad2deg(desired_heading))}, Dist to target:{round(dist_to_target)}")

    forward_vel_gain = 0.3
    angular_vel_gain = 0.5

    heading_correction = 0

    # if(dist_to_target < 0.1): # If we are close to the target, focus on correcting heading rather than moving forward.
    #     forward_vel_gain = 0

        #heading_correction = 0.1 * (to_pose[2] - from_pose[2]) # Add a correction term to help turn in place when close to the target.
    # COMMENT: Doesn't this mean at 10cm out, we stop moving?

    forward_vel = forward_vel_gain * dist_to_target    
    angular_vel = angular_vel_gain * heading_error + heading_correction
    forward_vel = np.clip(forward_vel, 0,0.2)
    angular_vel = np.clip(angular_vel, -1,1)
    return ((forward_vel, angular_vel), desired_heading)

def distance_to_wall_edge(pose, arena_size=(2, 2)):
    x, y, _ = pose
    width, height = arena_size

    distances = {
        "left": x,
        "right": width - x,
        "bottom": y,
        "top": height - y
    }

    nearest_wall = min(distances, key=distances.get)
    distance = distances[nearest_wall]

    return distance

def drive_to_line(current_pose, print_statements=True):
    # Line start and end points in world coordinates
    line_start = 0,2
    line_end = 2,0
    target = np.array(line_end)
    target_reached = False
    a = -2
    b = -2
    c = 4

    numerator = a*current_pose[0] + b*current_pose[1] + c
    denominator = np.sqrt(a**2 + b**2)

    distance_to_line = numerator / denominator
    if print_statements:
        print(f"Distance to line: {round(distance_to_line, 2)}, Wall license: {round(distance_to_wall_edge(current_pose), 2)}")

    heading_gain = 1
    distance_gain = 1.5
    forward_vel_max = 0.25
    min_wall_dist = 0.1
    forward_vel = np.clip(distance_to_wall_edge(current_pose) - min_wall_dist, 0, forward_vel_max) # Slow down as we get close to the wall.

    
    line_angle = np.arctan2(line_end[1] - line_start[1], line_end[0] - line_start[0])   

    line_angle_error = angle_diff(target=line_angle, current=current_pose[2]) * heading_gain
    distance_error = distance_to_line * distance_gain

    angular_vel = line_angle_error * heading_gain + distance_error * distance_gain
    limit = np.deg2rad(90)
    angular_vel = np.clip(angular_vel, -limit, limit)

    desired_heading = current_pose[2] + angular_vel * 0.5 # 0.5s timestep

    if print_statements:
        print(f"Line Angle: {round(np.rad2deg(line_angle))}, Line Angle Error: {round(np.rad2deg(line_angle_error))}, Distance Error: {round(distance_error, 2)}, Angular Vel: {round(angular_vel, 2)}")

    distance_to_target = np.linalg.norm(np.array([current_pose[0],current_pose[1]]) - target)
    if distance_to_target <= 0.15: # 15% within range
        target_reached = True
        


    return ((forward_vel, angular_vel), desired_heading, target_reached)

def pure_pursuit(current_pose):
    # Line start and end points in world coordinates
    line_start = 0,2
    line_end = 2,0

    look_ahead_distance = 0.2

    line_angle = np.arctan2(line_end[1] - line_start[1], line_end[0] - line_start[0])
    robot_to_line_start = np.linalg.norm(np.array(line_start) - np.array(current_pose[:2]))
    line_start_to_robot_angle = np.arctan2(current_pose[1] - line_start[1], current_pose[0] - line_start[0])
    robot_angle_to_line_angle = angle_diff(line_angle, line_start_to_robot_angle)
    pass

# def follow_bernoulli(current_pose, next_berno_point):
#     distance_to_target = np.linalg.norm(np.array([current_pose[0],current_pose[1]]) - next_berno_point)
#     desired_heading = np.arctan2(next_berno_point[1] - current_pose[1], next_berno_point[0] - current_pose[0])
#     heading_error = angle_diff(desired_heading, current_pose[2])
#     forward_vel_gain = 0.3
#     angular_vel_gain = 0.5
#     forward_vel = forward_vel_gain * distance_to_target
#     angular_vel = angular_vel_gain * heading_error
#     forward_vel = np.clip(forward_vel, 0,0.2)
#     angular_vel = np.clip(angular_vel, -1,1)
#     desired_heading = current_pose[2] + angular_vel * 0.5 # 0.5s timestep

#     return ((forward_vel, angular_vel), desired_heading)
def drive_to_segment(current_pose, line_start, line_end, print_statements=True):
    x1, y1 = line_start
    x2, y2 = line_end

    target = np.array(line_end)
    target_reached = False

    a = y1 - y2
    b = x2 - x1
    c = x1*y2 - x2*y1

    numerator = a*current_pose[0] + b*current_pose[1] + c
    denominator = np.sqrt(a**2 + b**2)

    distance_to_line = numerator / denominator

    line_angle = np.arctan2(y2 - y1, x2 - x1)

    heading_gain = 1.0
    distance_gain = 1.5
    forward_vel = 0.15

    line_angle_error = angle_diff(line_angle, current_pose[2])
    distance_error = distance_to_line

    angular_vel = heading_gain * line_angle_error + distance_gain * distance_error
    angular_vel = np.clip(angular_vel, -1.0, 1.0)

    desired_heading = line_angle

    distance_to_target = np.linalg.norm(np.array(current_pose[:2]) - target)
    if distance_to_target < 0.02:
        target_reached = True

    return (forward_vel, angular_vel), desired_heading, target_reached





def generate_bernoulli(center=(1.0, 1.0)):
    a = 1 / np.sqrt(2)

    t_values = np.linspace(0, 2 * np.pi, 200, dtype=np.float64)

    x_values = (a * np.cos(t_values)) / (np.sin(t_values)**2 + 1)
    y_values = (a * np.cos(t_values) * np.sin(t_values)) / (np.sin(t_values)**2 + 1)

    x_values = x_values + center[0]
    y_values = y_values + center[1]

    path = np.array([x_values, y_values], dtype=np.float64)
    return path 

