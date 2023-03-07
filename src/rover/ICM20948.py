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

class ICM20948:
        def __init__(self):
                self.filterIndex = 0
                self.filterSize = 10
                self.movingAverageX = [0.0] * self.filterSize
                self.movingAverageY = [0.0] * self.filterSize
                self.IMU = qwiic_icm20948.QwiicIcm20948()
                self.IMU.begin()

                self.ax_constant = 0
                self.ay_constant = 0

        def calibrate(self, con1, con2, con3):
                ACCEL = DECCEL = 1000
                GOTO_SPEED = 3000 #1750

                xmin = ymin = sys.maxsize #Hardcoded maximum size
                xmax = ymax = -sys.maxsize - 1 #Hardcoded minimum size

                #angle the wheels 
                con3.SpeedAccelDeccelPositionM2(RC_ADDR.FR, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
                con3.SpeedAccelDeccelPositionM2(RC_ADDR.BL, ACCEL, GOTO_SPEED, DECCEL, 300, 1)
                con2.SpeedAccelDeccelPositionM2(RC_ADDR.FL, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
                con2.SpeedAccelDeccelPositionM2(RC_ADDR.BR, ACCEL, GOTO_SPEED, DECCEL, -300, 1)
                sleep(2)

                turnCCW(30, con1, con2, con3)

                for _ in range(1500):
                        if self.IMU.dataReady():
                                self.IMU.getAgmt()
                                xmin = min(xmin, -self.IMU.mxRaw)
                                ymin = min(ymin, -self.IMU.myRaw)
                                xmax = max(xmax, -self.IMU.mxRaw)
                                ymax = max(ymax, -self.IMU.myRaw)
                                sleep(0.02)

                stop(con1, con2, con3)

                self.ax_constant = (xmax + xmin) / 2
                self.ay_constant = (ymax + ymin) / 2

                print(self.ax_constant, self.ay_constant)

        def getAngle(self):
                if self.IMU.dataReady():
                        self.IMU.getAgmt()
                        ax = -self.IMU.axRaw * 0.001 + 0.000
                        ay =  self.IMU.ayRaw * 0.001 + 0.010
                        az = -self.IMU.azRaw * 0.001 + 0.040
                        mx = -self.IMU.mxRaw - self.ax_constant
                        my = -self.IMU.myRaw - self.ay_constant
                        mz =  self.IMU.mzRaw - 9.50
                        """
                        roll = math.atan2(-ay, -az)
                        pitch = math.atan2(+ax, math.sqrt(ay * ay + az * az))
                        heading = math.atan2(-my * math.cos(roll) + mz * math.sin(roll),
                                                                mx * math.cos(pitch) + my * math.sin(pitch) * math.sin(roll) + mz *
                                                                math.sin(pitch) * math.cos(roll))
                """
                        self.movingAverageX[self.filterIndex] = mx
                        self.movingAverageY[self.filterIndex] = my
                        avgX = numpy.mean(self.movingAverageX)
                        avgY = numpy.mean(self.movingAverageY)
                        self.filterIndex = (self.filterIndex + 1) % self.filterSize
                        #return heading
                        return ((math.atan2(-avgY, avgX) * 180 / math.pi))%360
                else:
                        print("Waiting for data")
                        time.sleep(0.5)

def stop(con1, con2, con3):
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
def turnCCW(speed, con1, con2, con3):
        con3.ForwardM1(RC_ADDR.FR, speed)
        con2.ForwardM1(RC_ADDR.BR, speed)
        con2.ForwardM1(RC_ADDR.FL, speed)
        con3.ForwardM1(RC_ADDR.BL, speed)
        con1.ForwardM1(RC_ADDR.MID,speed)
        con1.ForwardM2(RC_ADDR.MID, speed)
def turnCW(speed, con1, con2, con3):
        con3.BackwardM1(RC_ADDR.FR, speed)
        con2.BackwardM1(RC_ADDR.BR, speed)
        con2.BackwardM1(RC_ADDR.FL, speed)
        con3.BackwardM1(RC_ADDR.BL, speed)
        con1.BackwardM1(RC_ADDR.MID, speed)
        con1.BackwardM2(RC_ADDR.MID, speed)

def tankTurn(speed, con1, con2, con3):
        if(speed > 0):          turnCW(abs(speed), con1, con2, con3)
        elif(speed < 0):        turnCCW(abs(speed), con1, con2, con3)

"""
tankTurnToAngle
        input 1: target angle (0-360)
        input 2: ICM20948 instantiation 
"""
def tankTurnToAngle(target, icm, con1, con2, con3):
        delta = 361
        deadZone = .5
        while (abs(delta) > deadZone):
                heading = icm.getAngle()
                delta = target - heading
                if delta > 180:         delta = delta - 360
                elif delta < -180:      delta = delta + 360
                tankTurn(int((delta / 3.0) + (abs(delta) / delta) * 20.0), con1, con2, con3)
                time.sleep(0.03)
        stop(con1, con2, con3)

def test(con1, con2, con3):
        icm = ICM20948()
        while(1):
                userInput = input("Enter target heading: ")
                if userInput == 'q':
                        break
                target = float(userInput)
                tankTurnToAngle(target, icm, con1, con2, con3)

if __name__ == '__main__':
        GPIO.setmode(GPIO.BOARD)

        con1 = Roboclaw("/dev/ttyS0", 115200, PowerGPIO.ML_MR)
        con2 = Roboclaw("/dev/ttyAMA1", 115200, PowerGPIO.FL_BR)
        con3 = Roboclaw("/dev/ttyAMA2", 115200, PowerGPIO.FR_BL)

        if con1.Open() == 0: raise UARTException(con1)
        if con2.Open() == 0: raise UARTException(con2)
        if con3.Open() == 0: raise UARTException(con3)

        try:
                test(con1, con2, con3)
        except (KeyboardInterrupt, SystemExit) as exErr:
                print("\nExiting")
                stop(con1, con2, con3)
                sys.exit(0)
