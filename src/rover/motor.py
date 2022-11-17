import threading
import time
from roboclaw_3 import Roboclaw
from util import RC_ADDR

CALIBRATION_TIME = 9
CALIBRATION_SPEED = 30
CALIBRATION_DELTA = 3
SLOWER_CALIBRATION_SPEED = 5

MAX_CORNER_ENC = 1550
INVALID_ENC = 1600

GOTO_SPEED = 1750#1750
GOTO_FR = 4542
ACCEL = DECCEL = 1000
ANGULAR_RANGE = 90  # was 72 for Herbie Mk1
COUNT_PER_METER = 28900

# test_data = []


class Motor:
    rc: Roboclaw
    rc_addr: int
    motor_ndx: int
    lock: threading.Lock

    def __init__(self, rc: Roboclaw, rc_addr: RC_ADDR, motor_ndx: int, lock: threading.Lock):
        self.rc = rc
        self.rc_addr = int(rc_addr)
        self.motor_ndx = motor_ndx
        self.lock = lock

    def go_to_position(self, position: int):
        with self.lock:
            if self.motor_ndx == 1:
                self.rc.SpeedAccelDeccelPositionM1(
                    self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
                )
            else:
                self.rc.SpeedAccelDeccelPositionM2(
                    self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
                )

    def move_distance(self, distance: int, speed: int = GOTO_SPEED):
        with self.lock:
            # if distance negative, make speed negative
            if distance < 0:
                speed *= -1
                distance *= -1
            if self.motor_ndx == 1:
                self.rc.SpeedDistanceM1(self.rc_addr, speed, distance, 1)
            else:
                self.rc.SpeedDistanceM2(self.rc_addr, speed, distance, 1)

    def move_distance_meters(self, distance: float, speed: float = GOTO_SPEED):
        """moves the motor a given distance in meters"""
        distance_enc = self.calculate_encoder_distance(distance)
        self.move_distance(distance_enc, speed)

    def encoder_value(self) -> int:
        with self.lock:
            if self.motor_ndx == 1:
                response = self.rc.ReadEncM1(self.rc_addr)
            else:
                response = self.rc.ReadEncM2(self.rc_addr)

            return response[1]

    def set_encoder_value(self, value: int) -> int:
        with self.lock:
            if self.motor_ndx == 1:
                response = self.rc.SetEncM1(self.rc_addr, value)
            else:
                response = self.rc.SetEncM2(self.rc_addr, value)

            return response[1]

    def stop(self):
        self.set_motor_register_speed('forward', 0)
        self.set_motor_register_speed('backward', 0)

    def set_motor_speed(self, direction: str, speed: float):
        """sets the speed of given a motor macro [WHEEL_FR, etc.] a direction and a speed (m/s)
            -max speed is 0.07 m/s
        """
        reg_speed = self.calculate_reg_speed(speed)
        self.set_motor_register_speed(direction, reg_speed)

    def set_motor_register_speed(self, direction: str, reg_speed: int):
        with self.lock:
            if reg_speed > 80:
                print("WARNING: speed to high %d" % reg_speed)
                return
            if direction == 'forward':
                if self.motor_ndx == 1:
                    self.rc.ForwardM1(self.rc_addr, reg_speed)
                else:
                    self.rc.ForwardM2(self.rc_addr, reg_speed)
            else:
                if self.motor_ndx == 1:
                    self.rc.BackwardM1(self.rc_addr, reg_speed)
                else:
                    self.rc.BackwardM2(self.rc_addr, reg_speed)

    def move_is_complete(self) -> bool:
        with self.lock:
            buffer = self.rc.ReadBuffers(self.rc_addr)
            if buffer[self.motor_ndx] == 128:
                return True
            return False

    @staticmethod
    def calculate_reg_speed(speed) -> int:
        result2 = (0.002 + speed) // 0.0009  # based on graph velocity formula for m/s
        result2 = int(result2)
        if result2 > 127:
            result2 = 127
        return result2  # velo = 0.0009(reg value) - 0.002

    @staticmethod
    def calculate_encoder_distance(distance: float) -> int:
        """calculates the encoder distance for a given distance in meters"""
        return int(distance * COUNT_PER_METER)


def CalibrateFailureException(Exception):
    def __init__(message: str):
        super.__init__(message)


class CornerMotor(Motor):
    rc: Roboclaw
    rc_addr: int
    motor_ndx: int
    lock: threading.Lock
    center: int
    left_most: int
    right_most: int
    calibrated: bool
    encoders_per_degree: int
    calibrate_fails: int

    def __init__(self, rc: Roboclaw, rc_addr: RC_ADDR, motor_ndx: int, lock: threading.Lock):
        super().__init__(rc, rc_addr, motor_ndx, lock)
        self.center = 0
        self.left_most = 0
        self.right_most = 0
        self.calibrated = False
        self.encoders_per_degree = 0  # ratio of encoder values
        self.calibrate_fails = 0

        with self.lock:
            self.rc.ResetEncoders(self.rc_addr)

    def __repr__(self) -> str:
        return "Addr: {}, left: {}, right: {}, calibrated?: {}".format(
            self.rc_addr, self.left_most, self.right_most, self.calibrated
        )

    def go_to_position(self, position: int) -> int:
        """Overrided to ensure position doesn't go out of bounds"""
        with self.lock:
            if not self.calibrated:
                print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
                return -1
            if position > self.right_most:
                print("position (%d) out of range, using rightmost instead" % position)
                position = self.right_most
            if position < self.left_most:
                print("position (%d) out of range, using leftmost instead" % position)
                position = self.left_most

            res: bool
            if self.motor_ndx == 1:
                res = self.rc.SpeedAccelDeccelPositionM1(
                    self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
                )
            else:
                res = self.rc.SpeedAccelDeccelPositionM2(
                    self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
                )
            return 0

    def go_to_left_most(self) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        self.go_to_position(self.left_most)
        return 0

    def go_to_right_most(self) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        self.go_to_position(self.right_most)
        return 0

    def go_to_center(self) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        self.go_to_position(self.center)
        return 0

    def rotate_n_degrees(self, direction, angle) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        encoder_distance = int(self.encoders_per_degree * angle)
        current_position = self.encoder_value()

        if direction == 'right':
            target_position = current_position + encoder_distance
        elif direction == 'left':
            target_position = current_position - encoder_distance
        else:
            print(f"Direction {direction} is not a valid way of turning")
            return -1
        print(f"Rotating {direction} for {angle} degrees")
        self.go_to_position(target_position)
        return 0

    def calibrate(self) -> 'tuple[int, int, int]':
        """Calibrates the motor by finding the leftmost and rightmost encoder values"""
        forward_found = 0

        for i in range(2):
            backward_found = 0
            prev_value = self.encoder_value()
            self.set_motor_register_speed('backward', CALIBRATION_SPEED)
            while(backward_found < 3):
                time.sleep(0.05)
                cur_value = self.encoder_value()
                if abs(cur_value - prev_value) < CALIBRATION_DELTA:
                    backward_found += 1
                prev_value = cur_value

            if i == 0:
                self.stop()
                self.move_distance(80)
            self.stop()

        self.left_most = self.encoder_value()

        for i in range(2):
            forward_found = 0
            prev_value = self.encoder_value()
            self.set_motor_register_speed('forward', CALIBRATION_SPEED)
            while(forward_found < 3):
                time.sleep(0.05)
                cur_value = self.encoder_value()
                if abs(cur_value - prev_value) < CALIBRATION_DELTA:
                    forward_found += 1
                prev_value = cur_value

            if i == 0:
                self.stop()
                self.move_distance(-80)
            self.stop()

        self.right_most = self.encoder_value()

        self.left_most += 50 if self.left_most < 0 else -50
        self.right_most += 50 if self.right_most < 0 else -50
        self.center = (self.left_most + self.right_most) // 2

        # # if we are not in the acceptable calibration range
        # if not (2300 < abs(self.right_most - self.left_most) < 2500):
        #     self._fails += 1
        #     if self.calibrate_fails <= 3:
        #         return self.calibrate()
        #     else:
        #         self.stop()
        #         raise CalibrateFailureException(
        #             f"Encoder on {self.rc_addr} failed to calibrate"
        #         )

        self.calibrated = True
        self.go_to_center()
        while not self.move_is_complete():
            time.sleep(0.05)
        self.stop()

        encoder_range = abs(self.right_most - self.left_most)
        self.encoders_per_degree = encoder_range // ANGULAR_RANGE

        # center the encoder
        self.left_most -= self.center
        self.right_most -= self.center
        self.center = 0
        self.rc.ResetEncoders(self.rc_addr)

        print(f"{self.rc_addr}: left: {self.left_most} right: {self.right_most} center: {self.center}")
        return (self.left_most, self.center, self.right_most)
