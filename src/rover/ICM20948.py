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
    #print("S" + str(speed))

    if(speed > 0):
        turnCW(abs(speed), con1, con2, con3)
    elif(speed < 0):
        turnCCW(abs(speed), con1, con2, con3)

def runExample():

	print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
	IMU = qwiic_icm20948.QwiicIcm20948()

	if IMU.connected == False:
		print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

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

	IMU.begin()
	i = 0
	size = 5
	movingAverageX = [0.0] * size
	movingAverageY = [0.0] * size
	maxX=-100
	minX=100

	maxY=-100
	minY=100
	while(1):
		userInput = input("Enter target heading: ")
		if userInput == 'q':
			break
		target = float(userInput)
		delta = 361
		deadzone = .5
		while (abs(delta)>deadzone):


			if IMU.dataReady():
				IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
				# print(\
				#  '{: 06d}'.format(IMU.axRaw)\
				# , '\t', '{: 06d}'.format(IMU.ayRaw)\
				# , '\t', '{: 06d}'.format(IMU.azRaw)\
				# , '\t', '{: 06d}'.format(IMU.gxRaw)\
				# , '\t', '{: 06d}'.format(IMU.gyRaw)\
				# , '\t', '{: 06d}'.format(IMU.gzRaw)\
				# , '\t', '{: 06d}'.format(IMU.mxRaw)\
				# , '\t', '{: 06d}'.format(IMU.myRaw)\
				# , '\t', '{: 06d}'.format(IMU.mzRaw)\
				# )

				ax = -IMU.axRaw * 0.001 + 0.000;
				ay = IMU.ayRaw * 0.001 + 0.010;
				az = -IMU.azRaw * 0.001 + 0.040;
				gx = -IMU.gxRaw - 0.400;
				gy = IMU.gyRaw - 0.730;
				gz = -IMU.gzRaw + 0.300;
				mx = -IMU.mxRaw - 136.00
				my = -IMU.myRaw - 1

				mz = IMU.mzRaw - 9.50;
				roll = math.atan2(-ay, -az);
				pitch = math.atan2(+ax, math.sqrt(ay * ay + az * az));
				heading = math.atan2(-my * math.cos(roll) + mz * math.sin(roll),
								mx * math.cos(pitch) + my * math.sin(pitch) * math.sin(roll) + mz * math.sin(pitch) * math.cos(roll));
				heading = ((heading*180/math.pi) + 270) % 360

				movingAverageX[i]=mx
				movingAverageY[i]=my
				avgX = numpy.mean(movingAverageX)
				avgY = numpy.mean(movingAverageY)
				if(avgX > maxX): maxX = avgX
				if(avgX < minX): minX = avgX
				if(avgY > maxY): maxY = avgY
				if(avgY < minY): minY = avgY
				heading = ((math.atan2(-avgY, avgX) * 180 / math.pi)-90)%360
				i=(i+1)%size

				print(
				'\t', '{: 06f}'.format(heading)
				, '\t', '{: 06f}'.format(avgX)
				, '\t', '{: 06f}'.format(avgY)
				, '\t', '{: 06f}'.format(maxX)
				, '\t', '{: 06f}'.format(minX)
				, '\t', '{: 06f}'.format(maxY)
				, '\t', '{: 06f}'.format(minY)
				)

				delta = target - heading
				if delta > 180:
					delta = delta - 360
				elif delta < -180:
					delta = delta + 360
				#int((delta / 10.0)
				tankTurn(int((delta / 3.0) + (abs(delta) / delta) * 20.0),con1, con2, con3)

				time.sleep(0.03)
			else:
				print("Waiting for data")
				time.sleep(0.5)
		stop(con1,con2,con3)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)