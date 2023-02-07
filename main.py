import math

def coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_theta_deg):
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    delta_theta_deg = 0
    if(delta_y < 0):
        delta_theta_deg = 180 + math.degrees(math.atan(float(delta_x) / float(delta_y))) - curr_theta_deg
    else:
        delta_theta_deg = math.degrees(math.atan(float(delta_x) / float(delta_y))) - curr_theta_deg
    return delta_theta_deg

def coords_to_target_distance(target_long, target_lat, curr_long, curr_lat):
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    return math.sqrt((delta_x**2)+(delta_y**2))