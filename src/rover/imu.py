"""
This is the folder to enable support for the IMU on I2C channel 1.
"""
import board
import adafruit_fxos8700
import adafruit_fxas21002c
import time


class Imu:
    def __init__(self):
        self.i2c = board.I2C()  # I2C channel 1
        self.fxos = adafruit_fxos8700.FXOS8700(self.i2c)  # accelerometer
        self.fxas = adafruit_fxas21002c.FXAS21002C(self.i2c)  # gyroscope

    def accelerometer(self):
        return self.fxos.accelerometer

    def magnetometer(self):
        return self.fxos.magnetometer

    def gyroscope(self):
        return self.fxas.gyroscope

    def printAccel(self):
        print(
            "Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
                *self.accelerometer()
            )
        )

    def printMag(self):
        print(
            "Magnetometer (uTesla): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
                *self.magnetometer()
            )
        )

    def printGyro(self):
        print(
            "Gyroscope (radians/s): ({0:0.3f},{1:0.3f},{2:0.3f})".format(
                *self.gyroscope()
            )
        )

    def Telementary(self, delay):
        while True:
            print(time.asctime())
            self.printAccel()
            self.printMag()
            self.printGyro()
            time.sleep(delay)