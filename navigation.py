import numpy as np
def navigate(from_pose, to_pose):
    # Simple heading controller
    dist_to_target = np.sqrt((to_pose[0] - from_pose[0])**2 + (to_pose[1] - from_pose[1])**2)
    desired_heading = np.arctan2(to_pose[1] - from_pose[1], to_pose[0] - from_pose[0])
    heading_error = desired_heading - from_pose[2]

    forward_vel_gain = 0.5
    angular_vel_gain = 1

    heading_correction = 0
    if(dist_to_target < 0.1): # If we are close to the target, focus on correcting heading rather than moving forward.
        forward_vel_gain = 0
        heading_correction = 3 * (to_pose[2] - from_pose[2]) # Add a correction term to help turn in place when close to the target.

    forward_vel = forward_vel_gain * dist_to_target    
    angular_vel = angular_vel_gain * heading_error + heading_correction
    return (forward_vel, angular_vel)