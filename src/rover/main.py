import math

from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, UARTException, RC_ADDR

import sys
from multiprocessing import Process, Pipe
sys.path.append("../../../Communications")
# from Communications.communications.parent import parent_proc
import communications as c

def parse_location(gps_location):
    c.send_packet(flag="EX_DONE", args=[])
    target_lat = gps_location[0:8]
    target_long = gps_location[8:]
    #fetch rover location
    curr_long = 2
    curr_lat = 1
    curr_theta_deg = 0

    #call brets calc functions
    deg_to_rotate = coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_theta_deg)
    distance_to_move = coords_to_target_distance(target_long, target_lat, curr_long, curr_lat)

    # need to do time calculations/calibration
    

    Move_Forward(distance_to_move)


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


# con1 = Roboclaw("/dev/ttyS0", 115200, PowerGPIO.ML_MR)
# if con1.Open() == 0:
#     raise UARTException(con1)

def move_middle_wheel_test(args):
    speed = int(args[0])
    con1.ForwardM2(RC_ADDR.MID, speed)

def Move_Forward(distance):
    speed = 50
    time = int(distance[0])/speed
    #con1.ForwardM2(RC_ADDR.MID, speed)
    forward(speed)
    time.sleep(time)
    forward(0)
    #con1.ForwardM2(RC_ADDR.MID, 0)

def forward(speed):
    con3.ForwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.FL, speed)
    con1.BackwardM1(RC_ADDR.MID, speed)
    con1.ForwardM2(RC_ADDR.MID, speed)
    con2.ForwardM1(RC_ADDR.BR, speed)
    con3.BackwardM1(RC_ADDR.BL, speed)


con1 = Roboclaw("/dev/ttyS0", 115200, PowerGPIO.ML_MR)
con2 = Roboclaw("/dev/ttyAMA1", 115200, PowerGPIO.FL_BR)
con3 = Roboclaw("/dev/ttyAMA2", 115200, PowerGPIO.FR_BL)
if con1.Open() == 0:
    raise UARTException(con1)
else:
    print("Successfully Connected")
if con2.Open() == 0:
    raise UARTException(con2)
else:
    print("Successfully Connected")
if con3.Open() == 0:
    raise UARTException(con3)
else:
    print("Successfully Connected")

def main():


    function_set = {
            "GPS": parse_location,
            "MOVE": Move_Forward 
        }

    rover_pipe, comms_pipe = Pipe()
    communications = Process(target=c.parent_proc, args=("192.168.4.1",7676, "192.168.4.3", 7777, function_set))
    communications.start()

    communications.join()

if __name__ == "__main__":
    main()
