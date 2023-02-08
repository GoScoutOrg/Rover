import math

from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, UARTException, RC_ADDR

import sys
from multiprocessing import Process, Pipe
sys.path.append("../../../Communications")
# from Communications.communications.parent import parent_proc
import communications



def parse_location(gps_location):
    send_packet(flag="EX_DONE", args=[])
    target_lat = gps_location[0:8]
    target_long = gps_location[8:]
    #fetch rover location
    curr_long = 2
    curr_lat = 1
    curr_theta_deg = 0

    #call brets calc functions
    deg_to_rotate = coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_theta_deg)
    distance_to_move = coords_to_target_distance(target_long, target_lat, curr_long, curr_lat)



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

def main():

    con1 = Roboclaw("/dev/ttyS0", 115200, PowerGPIO.ML_MR)
    if con1.Open() == 0:
        raise UARTException(con1)

    function_set = {
            "GPS": parse_location,
            "MOVE": con1.ForwardM1
        }

    rover_pipe, comms_pipe = Pipe()
    communications = Process(target=parent_proc, args=("192.168.4.3",7777, "192.168.4.10", 7676, function_set))
    communications.start()

    communications.join()

if __name__ == "__main__":
    main()
