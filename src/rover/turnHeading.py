from __future__ import print_function
import qwiic_icm20948
import time
import math
import sys
import numpy

from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, opposite_dir, UARTException, MotorPos, UART, RC_ADDR
import RPi.GPIO as GPIO
from time import sleep
import rover

CALIBRATION_TIME = 9
CALIBRATION_SPEED = 30
CALIBRATION_DELTA = 3
SLOWER_CALIBRATION_SPEED = 5

MAX_CORNER_ENC = 1550
INVALID_ENC = 1600

GOTO_SPEED = 3000#1750
GOTO_FR = 4542
ACCEL = DECCEL = 1000
ANGULAR_RANGE = 90  # was 72 for Herbie Mk1
COUNT_PER_METER = 28900

IMU = qwiic_icm20948.QwiicIcm20948()

def initIMU():
    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection",file=sys.stderr)
        return

    IMU.begin()

def updateHeading(index):
    #sleep(0.03)
    if IMU.dataReady():
        IMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables
        mx = -IMU.mxRaw - 5.00
        my = -IMU.myRaw - 2.50
        movingAverageX[index]=mx
        movingAverageY[index]=my
    heading1=((math.atan2(numpy.mean(movingAverageX), numpy.mean(movingAverageY)) * 360 / math.pi) + 90) % 360
    #heading = ((math.atan2(numpy.mean(mx), numpy.mean(my)) * 360 / math.pi) + 90) % 360
    return ((heading1 * 180 / math.pi) + 270) % 360

def stop():
    con3.BackwardM1(RC_ADDR.FR, 0)
    con2.ForwardM1(RC_ADDR.BR, 0)

    con1.BackwardM1(RC_ADDR.MID, 0)
    con1.ForwardM2(RC_ADDR.MID, 0)

    con2.ForwardM1(RC_ADDR.FL, 0)
    con3.BackwardM1(RC_ADDR.BL, 0)

    con3.BackwardM2(RC_ADDR.FR, 0)
    con2.ForwardM2(RC_ADDR.BR, 0)
    con2.ForwardM2(RC_ADDR.FL, 0)
    con3.BackwardM2(RC_ADDR.BL, 0)

def turnCCW(speed):
    con3.ForwardM1(RC_ADDR.FR, speed)
    con2.ForwardM1(RC_ADDR.BR, speed)

    con2.ForwardM1(RC_ADDR.FL, speed)
    con3.ForwardM1(RC_ADDR.BL, speed)

    con1.ForwardM1(RC_ADDR.MID,speed)
    con1.ForwardM2(RC_ADDR.MID, speed)

def turnCW(speed):
    con3.BackwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.BR, speed)

    con2.BackwardM1(RC_ADDR.FL, speed)
    con3.BackwardM1(RC_ADDR.BL, speed)

    con1.BackwardM1(RC_ADDR.MID, speed)
    con1.BackwardM2(RC_ADDR.MID, speed)

def tankTurn(speed):
    #print("S" + str(speed))
    if(speed > 0):
        turnCW(speed)
    elif(speed < 0):
        turnCCW(abs(speed))


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
stop()
# stop()
# sleep(1)
# r = rover.Rover()
#
# con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
# con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
# con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
# con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
sleep(1)
#stop()
deadzone = 10
index = 0
size = 10
movingAverageX = [0.0] * size
movingAverageY = [0.0] * size
current = 0.0
for i in range(size):
    current = updateHeading(index)
    index = (index + 1) % size
while(1):
    target = float(input("Enter target heading: "))
    delta  = 360
    while abs(delta) > deadzone:
        # int((delta/9.0)
        if IMU.dataReady():
            IMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables
            mx = -IMU.mxRaw - 5.00
            my = -IMU.myRaw - 2.50
            movingAverageX[index] = mx
            movingAverageY[index] = my
            index = (index + 1) % size
        heading = ((math.atan2(numpy.mean(movingAverageX), numpy.mean(movingAverageY)) * 360 / math.pi) + 90) % 360
        current = ((heading * 180 / math.pi) + 270) % 360
        delta = target - current
        if delta > 180:
            delta = delta - 360
        elif delta < -180:
            delta = delta + 360


        #tankTurn(int((abs(delta) / delta) * 15.0))
        print("H" + str(current))

        sleep(0.01)


    stop()
