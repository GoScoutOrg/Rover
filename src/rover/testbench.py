from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, opposite_dir, UARTException, MotorPos, UART, RC_ADDR
import RPi.GPIO as GPIO
from time import sleep
import rover

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

forward(100)

sleep(3)

backward(100)

sleep(3)

stop();

