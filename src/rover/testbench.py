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

def forward(speed):
    con3.ForwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.FL, speed)
    con1.BackwardM1(RC_ADDR.MID, speed)
    con1.ForwardM2(RC_ADDR.MID, speed)
    con2.ForwardM1(RC_ADDR.BR, speed)
    con3.BackwardM1(RC_ADDR.BL, speed)


def backward(speed):
    con3.BackwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.BR, speed)

    con2.ForwardM1(RC_ADDR.FL, speed)
    con3.ForwardM1(RC_ADDR.BL, speed)

    con1.ForwardM1(RC_ADDR.MID,speed)
    con1.BackwardM2(RC_ADDR.MID, speed)

def turn(speed):
    con3.ForwardM1(RC_ADDR.FR, speed)
    con2.ForwardM1(RC_ADDR.BR, speed)

    con2.ForwardM1(RC_ADDR.FL, speed)
    con3.ForwardM1(RC_ADDR.BL, speed)

    con1.ForwardM1(RC_ADDR.MID,speed)
    con1.ForwardM2(RC_ADDR.MID, speed)

def turn2(speed):
    con3.BackwardM1(RC_ADDR.FR, speed)
    con2.BackwardM1(RC_ADDR.BR, speed)

    con2.BackwardM1(RC_ADDR.FL, speed)
    con3.BackwardM1(RC_ADDR.BL, speed)

    con1.BackwardM1(RC_ADDR.MID, speed)
    con1.BackwardM2(RC_ADDR.MID, speed)

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



GPIO.setmode(GPIO.BOARD)



#r = rover.Rover()

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

# sleep(1)
# forward(30)

stop()
# sleep(10)
# con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
# con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
# con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
# con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, -300, 1)

#sleep(1)
##turn(30)
#forward(20)
#sleep(2)
#stop()
## sleep(.5)
## turn2(30)
## sleep(3)
## stop()
##
## con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, -330, 1)
## con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 220, 1)
## con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, -220, 1)
## con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, 330, 1)
##
## sleep(2)
##
## forward(20)
##
## sleep(4)
##
## backward(20)
##
## sleep(4)
##
## stop()
##
##
## con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
## con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
## con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
## con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, 0, 1)
##
## sleep(2)
##
## backward(30)
## sleep(4)
#
#stop()
