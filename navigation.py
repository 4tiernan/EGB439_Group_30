import numpy as np

def angle_diff(target, current):
    return (target - current + np.pi) % (2*np.pi) - np.pi

def navigate(from_pose, to_pose, constant_dirve = False, custom_gain = 0.3):
    # Simple heading controller
    dist_to_target = np.sqrt((to_pose[0] - from_pose[0])**2 + (to_pose[1] - from_pose[1])**2)
    desired_heading = np.arctan2( to_pose[1] - from_pose[1], to_pose[0] - from_pose[0])
    heading_error = angle_diff(desired_heading, from_pose[2])

    #print(f"Current Heading:{round(np.rad2deg(from_pose[2]))}")
    #print(f"Target Heading:{round(np.rad2deg(desired_heading))}, Dist to target:{round(dist_to_target)}")

    forward_vel_gain = custom_gain
    angular_vel_gain = 0.5

    heading_correction = 0
    if(dist_to_target < 0.1): # If we are close to the target, focus on correcting heading rather than moving forward.
        forward_vel_gain = 0
        #heading_correction = 0.1 * (to_pose[2] - from_pose[2]) # Add a correction term to help turn in place when close to the target.
    # COMMENT: Doesn't this mean at 10cm out, we stop moving?
    if constant_dirve:
        forward_vel = custom_gain
    else:
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

def drive_to_line(current_pose):
    # Line start and end points in world coordinates
    line_start = 0,2
    line_end = 2,0
    a = -2
    b = -2
    c = 4

    numerator = a*current_pose[0] + b*current_pose[1] + c
    denominator = np.sqrt(a**2 + b**2)

    distance_to_line = numerator / denominator
    #print(f"Distance to line: {round(distance_to_line, 2)}, Wall license: {round(distance_to_wall_edge(current_pose), 2)}")

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

    #print(f"Line Angle: {round(np.rad2deg(line_angle))}, Line Angle Error: {round(np.rad2deg(line_angle_error))}, Distance Error: {round(distance_error, 2)}, Angular Vel: {round(angular_vel, 2)}")

    return ((forward_vel, angular_vel), desired_heading)

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

def pure_pursuit(current_pose:np.ndarray, path:np.ndarray) -> np.ndarray:
    """
    pure_pursuit(robot_pose, desired_path)

    robot_pose -> The current pose of the robot in form [x, y, theta]

    desired_path -> The target path for the robot to follow 2xN

    Calculate direction velocity and angular velocity
    
    """

    #Step 0 - Adjust path to be [x,y],[x2,y2]
    x = path[0]
    y = path[1]
    path = np.column_stack((x,y))

    #Step 1 - Find closest point on path

    robot_pos = current_pose[0:2] # [x, y]

    closest_point = np.array([0, 0], dtype=np.float64)
    closest_point_distance = np.Infinity
    waypoint_index = -1
    line_scalar = 0

    for i in range(len(path)-1):
        current_waypoint = path[i]
        next_waypoint = path[i + 1]

        waypoint_scalar = closest_line(current_waypoint, next_waypoint, robot_pos)

        point = point_on_line(current_waypoint, next_waypoint, waypoint_scalar)

        distance = np.linalg.norm(point - robot_pos)

        if (distance < closest_point_distance):
            closest_point_distance = distance
            closest_point = point
            waypoint_index = i
            line_scalar = waypoint_scalar


    #Step 2 - Find point based on distance along path
    carrot_distance = 0.1

    goal = march_distance_along_path(path, waypoint_index, line_scalar, carrot_distance)


    #Step 3 - Calcuate heading to target point
    print(goal)
    return navigate(current_pose, np.array([goal[0], goal[1], 0]), True, 0.1)


def march_distance_along_path(path:np.ndarray, waypoint:int, scalar:float, distance:float) -> np.ndarray:
    """
        Marches along the path from the starting pos and returns the position on the line after a certain distance
    """
    remaining_distance = distance

    #Start step
    line_start = path[waypoint]
    line_end = path[waypoint + 1]

    line_length = np.linalg.norm(line_start - line_end)

    line_length = line_length * (1 - scalar) # get the remaining length of the line to the next waypoint

    if (line_length >= remaining_distance): # the marching is finished
        #Find the end point
        line_length = scalar + remaining_distance
        return point_on_line(line_start, line_end, line_length)
    
    remaining_distance -= line_length # Remove the line distance
    waypoint += 1
    #Run this proccess again with no scalar
    while True:
        line_start = path[waypoint]
        line_end = path[(waypoint + 1) % len(path)]

        line_length = np.linalg.norm(line_start - line_end)

        if (line_length >= remaining_distance): # the marching is finished
            #Find the end point
            line_length = remaining_distance
            return point_on_line(line_start, line_end, line_length)
        
        remaining_distance -= line_length # Remove the line distance
        waypoint += 1
        if waypoint >= len(path):
            waypoint = 0


def closest_point_line(start:np.ndarray, end:np.ndarray, point:np.array) -> np.ndarray:
    return point_on_line(start, end, closest_line(start, end, point))

def closest_line(start:np.ndarray, end:np.ndarray, point:np.ndarray) -> float:
    """
        Returns the scalar that results in a vector closest to the point
    """
    

    line_vector = end - start
    point_vector = point - start

    dot = np.dot(line_vector, point_vector)

    return dot


def point_on_line(start:np.ndarray, end:np.ndarray, scalar:float) -> np.ndarray:
    """
        Using scalar and line data to calculate the world position
    """

    line_vector = end - start

    return start + line_vector * scalar

def generate_bernoulli(center=(1.0, 1.0)):
    a = 1 / np.sqrt(2)

    t_values = np.linspace(0, 2 * np.pi, 200, dtype=np.float64)

    x_values = (a * np.cos(t_values)) / (np.sin(t_values)**2 + 1)
    y_values = (a * np.cos(t_values) * np.sin(t_values)) / (np.sin(t_values)**2 + 1)

    x_values = x_values + center[0]
    y_values = y_values + center[1]

    path = np.array([x_values, y_values], dtype=np.float64)
    return path 