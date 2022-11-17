import time
import threading
from typing import Union
import numpy as np
import RPi.GPIO as GPIO
from motor import Motor, CornerMotor
from roboclaw_3 import Roboclaw
from util import PowerGPIO, opposite_dir, UARTException, MotorPos, UART, RC_ADDR
from geometry import Arc, Point, point_to_angle
#from pac1720 import PAC1720


class Rover:
    """General class that contains all the information and methods needed to
    run the rover.
    """
    # all motors
    _motors: 'list[Union[Motor, CornerMotor]]'
    # all Roboclaws
    _uart_connections: 'list[Roboclaw]'
    # locks for each Roboclaw
    _locks: 'list[threading.Lock]'
    # movement motors
    _wheels: 'list[Motor]'
    # turning motors
    _corners: 'list[CornerMotor]'

    def __init__(self) -> None:
        """Initializes the rover. First it sets up the UART connections, then
        it sets up the motors and finally it calibrates the corners.
        """
        GPIO.setmode(GPIO.BOARD)

        # setup uart connections, may raise UARTException
        self.init_uart()

        # setup the power sensor
        #self._jetson_pi_sensor = PAC1720(6, 0x4a)  # jetson is channel 1, pi is channel 2
        #self._corners_sensor = PAC1720(6, 0x4f)  # FR_BL is channel 1, FL_BR is channel 2

        # setup the list of motors and initialize them
        self._motors = [
            Motor(self._uart_connections[UART._4], RC_ADDR.FL, 1, self._locks[UART._4]),  # front left wheel
            Motor(self._uart_connections[UART._5], RC_ADDR.FR, 1, self._locks[UART._5]),  # front right wheel
            Motor(self._uart_connections[UART._5], RC_ADDR.BL, 1, self._locks[UART._5]),  # back left wheel
            Motor(self._uart_connections[UART._4], RC_ADDR.BR, 1, self._locks[UART._4]),  # back right wheel
            Motor(self._uart_connections[UART._0], RC_ADDR.MID, 2, self._locks[UART._0]),  # middle left wheel
            Motor(self._uart_connections[UART._0], RC_ADDR.MID, 1, self._locks[UART._0]),  # middle right wheel
            CornerMotor(self._uart_connections[UART._4], RC_ADDR.FL, 2, self._locks[UART._4]),  # front left corner
            CornerMotor(self._uart_connections[UART._5], RC_ADDR.FR, 2, self._locks[UART._5]),  # front right corner
            CornerMotor(self._uart_connections[UART._5], RC_ADDR.BL, 2, self._locks[UART._5]),  # back left corner
            CornerMotor(self._uart_connections[UART._4], RC_ADDR.BR, 2, self._locks[UART._4])  # back right corner
        ]

        # get some more specific lists
        self._wheels = self._motors[MotorPos.FL:MotorPos.BR+1]
        self._corners = self._motors[MotorPos.C_FL:MotorPos.C_BR+1]

        # calibrate the corner motors
        threads = []
        for i in range(4):
            threads.append(threading.Thread(target=self._corners[i].calibrate))
            threads[-1].start()

        for thread in threads:
            thread.join()
        #self._uart_connections[2].SpeedAccelDeccelPositionM2(0x81, 32000, 12000, 32000, 0, 0)
        # self._corners[0].calibrate()
        # self._corners[2].calibrate()
        # self._corners[3].calibrate()
        # for motor in self._corners:
        #     motor.calibrate()
        #     threads.append(threading.Thread(target=motor.calibrate))
        #     threads[-1].start()
        #
        # for thread in threads:
        #     thread.join()
        
        breakpoint()

    def init_uart(self):
        # initialize the three uart locks
        self._locks = [
            threading.Lock(),
            threading.Lock(),
            threading.Lock()
        ]

        # initialize the three uart connections
        self._uart_connections = [
            Roboclaw("/dev/ttyS0", 115200, PowerGPIO.ML_MR),
            Roboclaw("/dev/ttyAMA1", 115200, PowerGPIO.FL_BR),
            Roboclaw("/dev/ttyAMA2", 115200, PowerGPIO.FR_BL)
        ]

        # open the connections and check if they opened successfully
        for conn in self._uart_connections:
            if conn.Open() == 0:
                raise UARTException(conn)
    
    def read_sensor(self):
        self._jetson_pi_sensor.begin()
        self._corners_sensor.begin()

        while True:
            print(f"Jetson:")
            print(f"\t{self._jetson_pi_sensor.get_sense_voltage(1):.2f}", end="")
            print(f" {self._jetson_pi_sensor.get_current(1):.2f}")

            print(f"Pi:")
            print(f"\t{self._jetson_pi_sensor.get_sense_voltage(2):.2f}", end="")
            print(f" {self._jetson_pi_sensor.get_current(2):.2f}")

            print(f"FR_BL:")
            print(f"\t{self._corners_sensor.get_sense_voltage(1):.2f}", end="")
            print(f" {self._corners_sensor.get_current(1):.2f}")

            print(f"FL_BR:")
            print(f"\t{self._corners_sensor.get_sense_voltage(2):.2f}", end="")
            print(f" {self._corners_sensor.get_current(2):.2f}")
            time.sleep(0.5)

    def wait_until_all_completed(self, subset: 'list[Motor]' = None):
        """Waits until all motors have completed their current move

        Args:
            subset (list[Motor], optional): The subset of motors to wait for.
                Defaults to None.
        """
        motors = self._motors

        if subset:
            motors = subset

        for motor in motors:
            while not motor.move_is_complete():
                time.sleep(0.5)
            motor.stop()

    def stop_motors(self, subset: 'list[Motor]' = None):
        """Stops all the motors or the ones specified

        Args:
            subset (list[Motor], optional): list of motors to stop.
            Defaults to None.
        """
        motors = subset if subset else self._motors

        for motor in motors:
            motor.stop()

    def recenter(self) -> int:
        """Moves all CornerMotors to their calibrated center

        Returns:
            int: 0 on success, -1 on failure
        """
        for motor in self._corners:
            motor.go_to_center()

        self.wait_until_all_completed()

    def move_straight(self, distance: float = 1):
        """Moves the rover forwards/backwards by a distance

        Args:
            distance (float, optional): distance to move in m. Defaults to 1.
        """
        for motor in self._wheels:
            motor.move_distance_meters(distance)

        self.wait_until_all_completed()

    def _set_arc_wheels(self, arc: Arc) -> None:
        """Turns corner wheels to position to make arc turn

        Args:
            arc (Arc): arc to turn
        """
        R_OUTER = 0.31  # 310 mm dist between front/back corner wheels
        R_HEIGHT = 0.556 / 2  # 278 mm dist between rover center and front

        self.recenter()

        # get the angle to turn
        outer_deg = np.rad2deg(np.arctan(R_HEIGHT / (arc.radius + R_OUTER / 2)))
        inner_deg = np.rad2deg(np.arctan(R_HEIGHT / (arc.radius - R_OUTER / 2)))

        # move the motors
        self._corners[0].rotate_n_degrees(
            arc.direction, outer_deg if arc.direction == 'right' else inner_deg
        )
        self._corners[1].rotate_n_degrees(
            arc.direction, outer_deg if arc.direction == 'left' else inner_deg
        )
        self._corners[0].rotate_n_degrees(
            opposite_dir(arc.direction),
            outer_deg if arc.direction == 'right' else inner_deg
        )
        self._corners[1].rotate_n_degrees(
            opposite_dir(arc.direction),
            outer_deg if arc.direction == 'left' else inner_deg
        )

        self.wait_until_all_completed()

    def _arc_turn_drive(self, arc: Arc) -> int:
        """Drives an arc turn by calcing the distance each wheel needs to go and
        modulating the speed accordingly

        Args:
            arc (Arc): arc to turn
        """
        # separation between turnable wheels and center wheels
        PRIMARY_OUTER_DIST = 0.0
        # separation between turnable wheels
        SECONDARY_DIST = 0.0

        # get the distance to travel for each wheel group
        # (close outer wheels, primary wheels, secondary wheels, far outer wheels)
        primary_dist = arc.arc_length()
        primary_out_dist = np.deg2rad(arc.angle) * (arc.radius + PRIMARY_OUTER_DIST)
        secondary_dist = np.deg2rad(arc.angle) * (arc.radius - SECONDARY_DIST)
        secondary_out_dist = np.deg2rad(arc.angle) * (arc.radius - (PRIMARY_OUTER_DIST + SECONDARY_DIST))

        def portion_of_max(dist: float):
            return dist / primary_out_dist

        # move the wheels
        if arc.direction == 'right':
            # going to the right so primary is left and secondary is right
            self._wheels[0].move_distance(primary_dist, portion_of_max(primary_dist))  # FL
            self._wheels[1].move_distance(secondary_dist, portion_of_max(secondary_dist))  # FR
            self._wheels[2].move_distance(primary_dist, portion_of_max(primary_dist))  # BL
            self._wheels[3].move_distance(secondary_dist, portion_of_max(secondary_dist))  # BR
            self._wheels[4].move_distance(primary_out_dist, portion_of_max(primary_out_dist))  # ML
            self._wheels[5].move_distance(secondary_out_dist, portion_of_max(secondary_out_dist))  # MR
        else:
            # going to the left so primary is right and secondary is left
            self._wheels[0].move_distance(secondary_dist, portion_of_max(secondary_dist))  # FL
            self._wheels[1].move_distance(primary_dist, portion_of_max(primary_dist))  # FR
            self._wheels[2].move_distance(secondary_dist, portion_of_max(secondary_dist))  # BL
            self._wheels[3].move_distance(primary_dist, portion_of_max(primary_dist))  # BR
            self._wheels[4].move_distance(secondary_out_dist, portion_of_max(secondary_out_dist))  # ML
            self._wheels[5].move_distance(primary_out_dist, portion_of_max(primary_out_dist))  # MR

        self.wait_until_all_completed()

    def arc_turn(self, arc: Arc) -> int:
        """Start an arc turn

        Args:
            direction (str): Which way will it turn, 'right' or 'left'
            arc (Arc): the arc to turn

        """
        if (0.45 > arc.radius or arc.radius > 300):
            print("radius must be between 0.45 and 300")
            return
        self._set_arc_wheels(arc.direction, arc.radius)
        return self._arc_turn_drive(arc.direction, arc)

    def m2p(self, endpoint: 'tuple[float]', midpoint: 'tuple[float]' = None):
        """Moves the rover to a point relative to it. May also arc.

        Args:
            endpoint (tuple[float]): (x, y) for the relative endpoint. Measure in meters.
            midpoint (tuple[float], optional): (x, y) for the midpoint of the arc. Defaults to None.
        """
        # if the midpoint exists, arc
        if midpoint:
            arc = Arc.from_points(Point(0, 0), Point(*midpoint), Point(*endpoint))
            self.arc_turn(arc)
        # otherwise, move straight with a turn
        else:
            angle = point_to_angle(endpoint)
            if angle != 0:
                self.turn('left', angle)
    
    def turn(self, direction: str, degrees: int):
        """Uses a tankturn to turn the rover a specific number of degrees

        Args:
            direction (str): 'left' or 'right'
            degrees (int): number of degrees to turn
        """

        # rotational speed in degrees per second
        ROT_SPEED = 12
        # max speed in m/s
        SPEED = 0.07

        if degrees == 0:
            return

        # move the corner motors into position
        self._motors[MotorPos.C_FL].go_to_right_most()
        self._motors[MotorPos.C_FR].go_to_left_most()
        self._motors[MotorPos.C_BL].go_to_left_most()
        self._motors[MotorPos.C_BR].go_to_right_most()

        self.wait_until_all_completed()

        # left is negative, right is positive
        direction = 'backward' if direction == 'left' else 'forward'

        for i in range(MotorPos.FL, MotorPos.MR+1):
            # left wheels
            if i % 2 == 0:
                # forward
                self._motors[i].set_motor_speed(direction, SPEED)
            # right wheels
            else:
                # backward
                self._motors[i].set_motor_speed(direction, SPEED)

        # wait until the motors have finished turning
        timeout = time.time() + (degrees / ROT_SPEED)
        while time.time() < timeout:
            time.sleep(0.02)

        self.stop_motors(self._wheels)

if __name__ == "__main__":
    r = Rover()