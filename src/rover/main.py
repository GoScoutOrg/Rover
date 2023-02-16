import math

from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, UARTException, RC_ADDR
import RPi.GPIO as GPIO
import gps_file as gps

import sys
from multiprocessing import Process, Pipe
sys.path.append("../../../Communications")
import communications as c

from time import sleep

#----------------------------------------------------------------#

#----------------------------------------------------------------#
GPIO.setmode(GPIO.BOARD)

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

#----------------------------------------------------------------#

#----------------------------------------------------------------#
my_gps = gps.setup_gps()
def parse_location(gps_location):
    # Coordinates: 35.3000821 N -120.66105479999999 
    # target_lat = gps_location[0:8]
    # target_long = gps_location[8:]
    target_lat = 35.3000821
    target_long = -120.66105479999999
    #fetch rover location

    curr_long = curr_lat = None;
    current_position = gps.get_gps(my_gps)
    if current_position:
        curr_long = current_position[1]
        curr_lat = current_position[0]
        # curr_theta_deg = 0
    else:
        print("ERROR")

    #call brets calc functions
    # deg_to_rotate = coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_theta_deg)
    distance_to_move = coords_to_target_distance(target_long, target_lat, curr_long, curr_lat)

    # need to do time calculations/calibration
    move_Forward(distance_to_move)

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
    METER_TO_COORD_DEG_RATIO = 111139
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    return math.sqrt((delta_x**2)+(delta_y**2)) * METER_TO_COORD_DEG_RATIO

def move_Forward(distance):
    speed = 50
    time = distance/speed
    forward(speed)
    sleep(time)
    forward(0)

def forward(speed):
    con3.ForwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.FL, speed)
    con1.BackwardM1(RC_ADDR.MID, speed)
    con1.ForwardM2(RC_ADDR.MID, speed)
    con2.ForwardM1(RC_ADDR.BR, speed)
    con3.BackwardM1(RC_ADDR.BL, speed)

def main():
    function_set = {
            "GPS": parse_location,
            "MOVE": move_Forward 
        }

    rover_pipe, comms_pipe = Pipe()
    communications = Process(target=c.parent_proc, args=("192.168.4.1",7676, "192.168.4.3", 7777, function_set))
    communications.start()

    communications.join()

if __name__ == "__main__":
    main()
