import math

import rover
from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, UARTException, RC_ADDR
import RPi.GPIO as GPIO
import gps_file as gps
import ICM20948 as icm_file
import rover 

import sys
from multiprocessing import Process, Pipe
sys.path.append("../../../Communications")
import communications as c

from time import sleep

GOTO_SPEED = 3000 #1750
ACCEL = DECCEL = 1000

#----------------------------------------------------------------#
#INIT 
#----------------------------------------------------------------#

#init icm 
icm = icm_file.ICM20948()
GPIO.setmode(GPIO.BOARD)
#init roboclaws
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

#init rover calibration 
r = rover.Rover()
#----------------------------------------------------------------#
#GPS FUNCTIONALITY
#----------------------------------------------------------------#

def send_ack():
    c.send_packet(flag = "ROVER_ACK", data= []) 

my_gps = gps.setup_gps()
def parse_location(gps_location):
    ##############
    # STEPS: 
    #     1. RECIEVE GPS 
    #     2. DETERMINE OWN GPS 
    #     3. CALCULATE distance and degrees orientation
    #     4. call function for movement
    ##############

    #coords: 35.3001594 N -120.6610097 
    # target_lat = gps_location[0:8]
    # target_long = gps_location[8:]

    #STEP 1
    # target_lat = 35.3001594
    # target_long = -120.6610097

    vals = gps_location[0].split(':')
    print(vals)
    target_lat = float(vals[0])
    target_long = float(vals[1])

    #STEP 2
    #fetch rover location
    curr_long = curr_lat = None
    current_position = gps.get_gps(my_gps)
    if current_position:
        curr_long = current_position[1]
        curr_lat = current_position[0]
        # curr_theta_deg = 0
    else:
        print("ERROR")
    #STEP 3
    distance_to_move = gps_to_meters(target_lat, target_long, curr_lat, curr_long)
    print("moving: ", distance_to_move)
    forward = centimeters_to_forward(distance_to_move)
    print("moving: ", forward)

    #STEP 4
    do_tank_turn(target_long, target_lat, curr_long, curr_lat)

    # need to do time calculations/calibration
    move_forward(forward)
#----------------------------------------------------------------#

#CALCULATIONS
#----------------------------------------------------------------#

def coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_theta_deg):
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    delta_theta_deg = 0
    if(delta_y < 0):
        delta_theta_deg = 180 + math.degrees(math.atan(float(delta_x) / float(delta_y))) - curr_theta_deg
    else:
        delta_theta_deg = math.degrees(math.atan(float(delta_x) / float(delta_y))) - curr_theta_deg
    return delta_theta_deg

def delta_theta_to_global_target_direction(curr_orientation, delta_theta_deg):
    global_target_direction = curr_orientation + delta_theta_deg
    return global_target_direction % 360

def coords_to_target_distance(target_long, target_lat, curr_long, curr_lat):
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    return math.sqrt((delta_x**2)+(delta_y**2))
    
def gps_to_meters(lat1, lon1, lat2, lon2):
    earth_radius = 6378.137 #KM
    dlat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dlon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = earth_radius * c
    return d * 100000 # centimeters

def centimeters_to_forward(x):
    return 1.56 * x - 2.04
#----------------------------------------------------------------#
#MOVEMENT
#----------------------------------------------------------------#

def move_forward(distance):
    speed = 50
    time = distance/speed
    forward(speed)
    sleep(time)
    forward(0)
    send_ack()

def forward(speed):
    con3.ForwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.FL, speed)
    con1.BackwardM1(RC_ADDR.MID, speed)
    con1.ForwardM2(RC_ADDR.MID, speed)
    con2.ForwardM1(RC_ADDR.BR, speed)
    con3.BackwardM1(RC_ADDR.BL, speed)

def do_tank_turn(target_long, target_lat, curr_long, curr_lat):
    #angle the wheels 
    con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
    con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
    con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
    con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
    sleep(2)

    curr_orientation = 0
    for i in range(10):
        icm.getAngle()
        if i == 9:
            curr_orientation = icm.getAngle()

    delta_theta_deg = coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_orientation)
    print("Turning: ", delta_theta_deg)

    target = delta_theta_to_global_target_direction(curr_orientation, delta_theta_deg)
    icm_file.tankTurnToAngle(target, icm, con1, con2, con3)
    #unangle the wheels
    con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
    con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
    con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
    con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, 0, 1)

    sleep(2)
#----------------------------------------------------------------#
#MAIN 
#----------------------------------------------------------------#

def main():
    move_forward(0)
    function_set = {
            "GPS": parse_location,
            "MOVE": move_forward 
        }
    rover_pipe, comms_pipe = Pipe()

    communications = Process(target=c.parent_proc, args=("192.168.4.1", 7676, "192.168.4.10", 7777, function_set))

    communications.start()

    # debugging IMU orientation
    while(True):
        print("Current orientation: ", icm.getAngle())
        sleep(0.2)


    communications.join()

if __name__ == "__main__":
    main()


#----------------------------------------------------------------#
