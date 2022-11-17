import time
import math
import threading
import sys
from typing import Union
import rover.motor as motor
import rover.roboclaw_3 as roboclaw_3
from rover.geometry import Point, Line, Arc

""" GPIO setup for follow me mode:
    Since the I2C pins are set up using the BCM option
    instead of the BOARD option, BCM must be used for the
    entire program for consistency. This is why pin numbers
    are different on the RP4 as opposed to the Jetson Nano.
"""

# certain type hints need both types of Motor, use this
T = Union[motor.Motor, motor.CornerMotor]

# arc turn constants
R_OUTER = 0.31  # 310 mm dist between front/back corner wheels
R_OUTER_MID = 0.4  # 400 mm dist between center drive wheels
R_HEIGHT = 0.556 / 2  # 278 mm dist between rover center and front
MAX_TURN = 36
MAX_SPEED_VEL = 0.05
CALIBRATION_SPEED = 30
STOP_THRESHOLD = 100
MOVING_THRESHOLD = 50
SLEEP_TIME = 0.25
COUNT_PER_METER = 28900

# number of seconds per degree for tank turn
# SECONDS_PER_DEGREE = 0.12722222
SECONDS_PER_DEGREE = 39.8 / 360

# direction of turning
# 0 is no turning, 1 is clockwise, -1 is counter-clockwise
TURN_DIRECTION = 0


#uart0 is the front wheels
#uart4 is the rear wheels
#uart5 is the middle  wheels
uart0 = roboclaw_3.Roboclaw("/dev/ttyS0", 115200)
uart4 = roboclaw_3.Roboclaw("/dev/ttyAMA1", 115200)
uart5 = roboclaw_3.Roboclaw("/dev/ttyAMA2", 115200) #THIS MIGHT NOT BE THE CORRECT TTY port

errors = [uart0.Open(), uart4.Open(), uart5.Open()]
for i, error in enumerate(errors):
    if error == 0:
        print(f"Uart at position {i} failed.")

if sum(errors) != 3:
    sys.exit()


RC_ADDR_BR = 0x82 #uart4 AMA1
RC_ADDR_FL = 0x80 #uart4
RC_ADDR_FR = 0x81 #uart5 AMA2
RC_ADDR_BL = 0x83 #uart5
RC_ADDR_MID = 0x84 #uart0

# Driving motors
#uart0

lock0 = threading.Lock()
WHEEL_ML = motor.Motor(uart0, RC_ADDR_MID, 2, lock0)  # (rc_addr, mi or m2)
WHEEL_MR = motor.Motor(uart0, RC_ADDR_MID, 1, lock0)

#uart4
lock4 = threading.Lock()
WHEEL_FL = motor.Motor(uart4, RC_ADDR_FL, 1, lock4)  # (rc_addr, mi or m2)
WHEEL_BR = motor.Motor(uart4, RC_ADDR_BR, 1, lock4)  # (rc_addr, mi or m2)
CORNER_BR = motor.CornerMotor(uart4, RC_ADDR_BR, 2, lock4)
CORNER_FL = motor.CornerMotor(uart4, RC_ADDR_FL, 2, lock4)

#uart5
lock5 = threading.Lock()
WHEEL_FR = motor.Motor(uart5, RC_ADDR_FR, 1, lock5)  # (rc_addr, mi or m2)
WHEEL_BL = motor.Motor(uart5, RC_ADDR_BL, 1, lock5)  # (rc_addr, mi or m2)
CORNER_FR = motor.CornerMotor(uart5, RC_ADDR_FR, 2, lock5)
CORNER_BL = motor.CornerMotor(uart5, RC_ADDR_BL, 2, lock5)

UART4Corner: list = [CORNER_FL, CORNER_BR] #just the corners on UART4
UART5Corner: list = [CORNER_FR, CORNER_BL] #just the corners on UART5
CORNERS: list = [CORNER_FL, CORNER_FR, CORNER_BL, CORNER_BR]

WHEELS: list = [WHEEL_FL, WHEEL_FR, WHEEL_ML, WHEEL_MR, WHEEL_BL, WHEEL_BR]
WHEELS_LEFT: list = [WHEEL_FL, WHEEL_ML, WHEEL_BL]
WHEELS_RIGHT: list = [WHEEL_FR, WHEEL_MR, WHEEL_BR]

ALL_MOTORS: dict = {
    "wheel_fr": WHEEL_FR,
    "wheel_fl": WHEEL_FL,
    "wheel_ml": WHEEL_ML,
    "wheel_mr": WHEEL_MR,
    "wheel_bl": WHEEL_BL,
    "wheel_br": WHEEL_BR,
    "corner_fl": CORNER_FL,
    "corner_fr": CORNER_FR,
    "corner_bl": CORNER_BL,
    "corner_br": CORNER_BR,
}

# helpers
def set_speed_left_side(direction: str, cen_speed: float, speed: float = None) -> None:
    """sets all left side wheels to given a direction [forward, backward] and speed (m/s)
    Center wheel speed is different to allow arc turns

    Args:
        direction (str): 'forward' or 'backward'
        cen_speed (float): speed of the center wheels (m/s)
        speed (float, optional): speed of the corner wheels (m/s). Defaults to None.
    """
    if speed is None:
        speed = cen_speed
    for wheel in WHEELS_LEFT:
        if wheel is WHEEL_ML:
            wheel.set_motor_speed(direction, cen_speed)
        else:
            wheel.set_motor_speed(direction, speed)


def set_speed_right_side(direction: str, cen_speed: float, speed: float = None) -> None:
    """sets all right side wheels to given a direction [forward, backward] and speed (m/s)
    Center wheel speed is different to allow arc turns

    Args:
        direction (str): 'forward' or 'backward'
        cen_speed (float): speed of the center wheels (m/s)
        speed (float, optional): speed of the corner wheels (m/s). Defaults to None.
    """
    if speed is None:
        speed = cen_speed
    for wheel in WHEELS_RIGHT:
        if wheel is WHEEL_MR:
            wheel.set_motor_speed(direction, cen_speed)
        else:
            wheel.set_motor_speed(direction, speed)


def set_speed_all(direction: str, speed: float = 0.05) -> None:
    """sets all the wheels to either a specified value or about 0.1 m/s

    Args:
        direction (str): 'forward' or 'backward'
        speed (float, optional): speed to be used in Motor.set_motor_speed() (m/s).
            Defaults to 0.05.
    """
    set_speed_left_side(direction, speed)
    set_speed_right_side(direction, speed)

def stop_all_wheels() -> None:
    """Stops all drive wheels on call
    """
    for motor in WHEELS:
        motor.stop()

def get_time(speed, dist) -> float:
    """Simple time = dist / speed calculation

    Args:
        speed (float): velocity (m/s)
        dist (float): distance (m)

    Returns:
        float: time (s)
    """
    howLong = abs(dist / speed)  # velo = distance / time
    return howLong


# for arc turns
def get_inner_velo(radius: float, outer_speed: float) -> tuple:
    """Return speed for inner drive wheels

    Args:
        radius (float): radius of the arc
        outer_speed (float): speed of the center wheel on the outside of the arc (m/s)

    Returns:
        tuple (float, float): val for motor address 0x81 in m/s,
                                val for motor address 0x84 in m/s
    """
    inner_velo = outer_speed * (
        math.sqrt(radius ** 2 - 0.31 * radius + 0.1013) / (radius + 0.2)
    )
    inner_cen_velo = outer_speed * ((radius - 2) / (radius + 0.2))
    return (inner_velo, inner_cen_velo)


## commands
def rotate(motor_name: str, direction: str, angle: float) -> int:
    """Rotate a single CornerMotor given degrees in a given directoin

    Args:
        motor_name (str): CornerMotor name
        direction (str): 'left' or 'right'
        angle (float): angle in degrees to turn the wheel

    Returns:
        int: 0 on success, -1 otherwise
    """
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None:
        print(f"Invalid motor name: {motor_name}")
        return -1
    if not isinstance(mtr, motor.CornerMotor):
        print(f"Invalid motor type: {type(mtr).__name__}")
        return -1
    angle = int(angle)
    mtr.rotate_n_degrees(direction, angle)
    return 0

def get_encoders():
    return [(k, v.encoder_value()) for k, v in ALL_MOTORS.items()]

def print_encoders() -> None:
    """Prints out all the encoder register values"""
    for name, value in get_encoders():
        print(f"{name}: {value}")


def position(motor_name: str, position: int) -> int:
    """Moves a single wheel to a certain encoder value

    Args:
        motor_name (str): name of the Motor
        position (int): encoder value to rotate to

    Returns:
        int: 0 on success, -1 on errors
    """
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None:
        print(f"Invalid motor name: {motor_name}")
        return -1
    position = int(position)
    mtr.go_to_position(position)
    return 0


def move_distance_meters(motor_name: str, distance: float) -> int:
    """Moves a singular motor, or all motors, a certain distance in meters

    *** TODO: implement this but with the IMU

    Args:
        motor_name (str): Motor name or 'all'
        distance (float): distance (m)

    Returns:
        int: 0 on success, -1 on error
    """
    recenter()
    distance = float(distance)
    # radius of a wheel is roughly 6cm
    # so 1 meter divided by circumference of wheel 
    # multiplied by encoders per revolution (10900)
    if motor_name == "all":
        for wheel in WHEELS:
            encoder_dist = int(distance * COUNT_PER_METER)
            wheel.move_distance(encoder_dist, int(1750 * 1.5))
        # wait until move complete
        # chose MR randomly, but should be representative

        # wait_until_move_complete(WHEEL_BR)
        wait_until_all_complete()
        stop_all_wheels()
    else:
        mtr = ALL_MOTORS.get(motor_name, None)
        if mtr is None:
            print(f"Invalid motor name: {motor_name}")
            return -1
        if isinstance(mtr, motor.CornerMotor):
            print(f"Invalid motor type: {type(mtr).__name__}")
            return -1
        encoder_dist = int(distance * COUNT_PER_METER)
        mtr.move_distance(encoder_dist, int(1750 * 1.5))

        # wait until move complete
        wait_until_move_complete(mtr)
    return 0


def wheel_stats(motor_name: str) -> int:
    """Print the leftmost, center, and rightmost encoder value for a motor

    *** TODO: This needs to get moved in to motor.py or removed

    Args:
        motor_name (str): name of a motor

    Returns:
        int: 0 on success, -1 on error
    """
    mtr = ALL_MOTORS.get(motor_name.lower(), None)
    if mtr is None:
        print(f"Invalid motor name: {motor_name}")
        return -1
    if isinstance(mtr, motor.CornerMotor):
        print(f"Left: {mtr.left_most}, Center: {mtr.center}, Right: {mtr.right_most}")
        return 0
    print("No stats to print for a drive motor.")
    return 0


def get_turning() -> int:
    """Checks TURN_DIRECTION variable
    
    Returns:
        int: Direction of turning (0 for no turning, 1 for clockwise, 
             -1 for counter-clockwise)
    """
    return TURN_DIRECTION


def calibrate_all() -> int:
    """Calibrates the CornerMotors using threads

    Returns:
        int: 0, nothing in this can return a failure
    """
    thrds: list[threading.Thread] = []
    #to do wheel by the UART channel
    for corner in CORNERS:
        thrds.append(threading.Thread(target=corner.calibrate))
        thrds[-1].start()

    for thread in thrds:
        thread.join()

    for cr in CORNERS:
        cr.stop()

    return 0


def calibrate_one(motor_name: str) -> int:
    """Calibrate a single CornerMotor, no multi-threading

    Args:
        motor_name (str): name of the CornerMotor

    Returns:
        int: 0 on success, -1 on error
    """
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None:
        print(f"Invalid motor name: {motor_name}")
        return -1
    if isinstance(mtr, motor.CornerMotor):
        mtr.calibrate()
        return 0
    else:
        print(f"Invalid motor type: {type(mtr).__name__}")
        return -1


def kill_all() -> int:
    """Stops all motors
    """
    for wheel in ALL_MOTORS.values():
        wheel.stop()
    return 0


def rotate_max(motor_name: str, direction: str) -> int:
    """Rotate a single CornerWheel to its max value

    Args:
        motor_name (str): CornerMotor name
        direction (str): 'left' or 'right'

    Returns:
        int: 0 on success, -1 on error
    """
    direction = direction.lower()
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None:
        print(f"Invalid motor name: {motor_name}")
        return -1
    if not isinstance(mtr, motor.CornerMotor):
        print(f"Invalid motor type: {type(mtr).__name__}")
        return -1
    if direction == "right":
        return mtr.go_to_right_most()
    elif direction == "left":
        return mtr.go_to_left_most()
    else:
        print(f"Direction {direction} is not a valid way of rotating")
        return -1


def recenter() -> int:
    """Moves all CornerMotors to their calibrated center

    Returns:
        int: 0 on success, -1 on failure
    """
    if not all([corner.calibrated for corner in CORNERS]):
        calibrate_all()
        return -1
    for corner in CORNERS:
        corner.go_to_center()

    # chose FL randomly, but should be representative
    # wait_until_move_complete(CORNER_BR)
    wait_until_all_complete()
    for corner in CORNERS:
        corner.stop()
    return 0


def wait_until_all_complete() -> None:
    """Waits until every motor is done moving before returning
    """
    while True:
        if all([mtr.move_is_complete() for mtr in ALL_MOTORS.values()]):
            break
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def wait_until_move_complete(motor: motor.Motor) -> None:
    """Waits until a specific motor is done moving before returning

    Args:
        motor (motor.Motor): some motor that we are waiting for
    """
    while not motor.move_is_complete():
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def wait_until_position(position: int, motor: motor.Motor) -> None:
    """Waits until a specific motor moves withing a certain threshold

    *** TODO: unused?

    Args:
        position (int): encoder value to wait until
        motor (motor.Motor): motor to wait for
    """
    current = motor.encoder_value()
    diff = abs(current - position)
    while diff > STOP_THRESHOLD:
        current = motor.encoder_value()
        diff = abs(current - position)
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def wait_until_stopped(motor) -> None:
    """Waits until a specific motor stops

    *** TODO: unused?

    Args:
        motor (motor.Motor): motor to wait for
    """
    current = motor.encoder_value()
    time.sleep(SLEEP_TIME)
    new = motor.encoder_value()
    while abs(current - new) > MOVING_THRESHOLD:
        current = new
        new = motor.encoder_value()
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def degrees_to_duration(degrees: float) -> tuple:
    """If degrees is less than or equal to 0, returns 0.
    Will return a mod 360 value, so wheels don't try to make multiple revolutions

    Args:
        degrees (float): degrees to turn

    Returns:
        float: duration the wheel will be turned (s)
        float: degree value modified (0 or greater)
    """
    degrees = float(degrees)  # probably not necessary
    if degrees < 0:
        degrees = 0  # minimum of 0 degrees
    elif degrees > 360:
        degrees = degrees % 360  # convert to within 360 degrees
    duration = float(degrees * SECONDS_PER_DEGREE)  # convert degrees to time
    return (duration, degrees)  # return duration and possibly converted degrees


def distance_tank(direction: str, degrees: float) -> int:
    """Executes the actual wheel spinning to caues rover to rotate

    Args:
        direction (str): 'left' or 'right'
        degrees (float): number of degrees to spin

    Returns:
        int: 0 on success, -1 on error
    """
    direction = direction.lower()
    # check tank_turn.pdf for this value
    rad: float = 0.2122
    # want to make sure degrees makes sense, but duration is ignorable
    _, degrees = degrees_to_duration(degrees)
    if degrees == 0:
        print(
            "Attempted to turn 0 or negative amount of degrees, input positive values only"
        )
        return -1
    print(f"Tank mode: {direction} for {degrees:.2f} degrees")
    if direction == "right":
        TURN_DIRECTION = 1
        # set speed
        for wheel in WHEELS_LEFT:
            wheel.move_distance(int(rad * degrees) * COUNT_PER_METER // 36)
        for wheel in WHEELS_RIGHT:
            wheel.move_distance(int(-1 * rad * degrees) * COUNT_PER_METER // 36)
        # wait for duration
        wait_until_all_complete()
        # stop motors
        for wheel in WHEELS:
            wheel.stop()
        TURN_DIRECTION = 0
        return 0
    elif direction == "left":
        TURN_DIRECTION = -1
        for wheel in WHEELS_LEFT:
            wheel.move_distance(int(-1 * rad * degrees) * COUNT_PER_METER // 36)
        for wheel in WHEELS_RIGHT:
            wheel.move_distance(int(rad * degrees) * COUNT_PER_METER // 36)
        # wait for duration
        wait_until_all_complete()
        # stop motors
        for wheel in WHEELS:
            wheel.stop()
        TURN_DIRECTION = 0
        return 0
    else:
        print(f"Direction {direction} is not a supported direction")
        return -1



def tank_align() -> int:
    """Just do the turn part of the tank turn

    Returns:
        int: 0 on success, -1 on errors. Prints error msg to stdout
    """
    print("Turning wheels")
    print("front right")
    CORNER_FR.go_to_left_most()
    print("front left")
    CORNER_FL.go_to_right_most()
    print("back right")
    CORNER_BR.go_to_right_most()
    print("back left")
    CORNER_BL.go_to_left_most()
    wait_until_move_complete(CORNER_BL)
    for cr in CORNERS:
        cr.stop()

    return 0


def tank_with_turn(direction: str, degrees: float) -> int:
    """Sets the CornerWheels to proper alignment to do the tankturn

    Args:
        direction (str): 'left' or 'right'
        degrees (float): number of degrees to turn

    Returns
        int: 0 on success, -1 on error
    """
    if tank_align() < 0:
        return -1
    else:
        return distance_tank(direction, degrees)


def forward(speed: float = 0.05, dist: float = 1) -> int:
    """Drives rover straight forward at specified speed and distance

    *** TODO: unused?

    Args:
        speed (float, optional): velocity (m/s). Defaults to 0.05.
        dist (float, optional): distance (m). Defaults to 1.

    Returns:
        int: 0 on success, -1 on error
    """
    speed = float(speed)
    dist = float(dist)

    if dist <= 0:
        print(f"Invalid distance entered: {dist:.2f}. Positive values only.")
        return -1
    if speed <= 0:
        print(f"Invalid speed entered: {speed:.2f}. Positive values only.")

    duration = dist / speed

    print(
        f"Driving forward at {speed:.4f} m/s for {dist:.2f} meters for {duration:.2f} seconds"
    )

    for wheel in WHEELS:
        wheel.set_motor_speed('forward', speed)

    time.sleep(duration)

    for wheel in WHEELS:
        wheel.stop()
    return 0


def forward_with_stop(speed: float) -> int:
    """Drives rover straight forward at specified speed until user says to stop
    Input [y/yes] for it to stop driving.

    NOTE: This should only be used for debugging

    Args:
        speed (float): velocity (m/s)

    Returns:
     int: 0 no matter what
    """
    speed = float(speed)

    print("Driving forward at %.4f m/s" % (speed))
    direction = "forward"

    for wheel in WHEELS:
        wheel.set_motor_speed(direction, speed)

    # instead of waiting for specific input, easier for user to just press Enter
    # input will block until Enter pressed anyway
    input("Press Enter to stop! ").lower()

    stop_all_wheels()
    return 0


def backward(speed: float, dist: float) -> int:
    """Drives rover straight backward at specified speed for specified distance

    *** TODO: unused?

    Args:
        speed (float): velocity (m/s)
        dist (float): distance (m)

    Returns:
        int: 0 on success, -1 on error. Prints error message to stdout
    """
    speed = float(speed)
    dist = float(dist)

    if dist <= 0:
        print(f"Invalid distance entered: {dist:.2f}")
        return -1
    if speed <= 0:
        print(f"Invalid speed entered: {speed:.2f}")
        return -1

    howLong = get_time(speed, dist)

    print(
        f"Driving backward at {speed:.4f} m/s for {dist:.2f} meters or {howLong:.2f} seconds"
    )

    for wheel in WHEELS:
        wheel.set_motor_speed("backward", speed)
    time.sleep(howLong)
    for wheel in WHEELS:
        wheel.stop()
    return 0


def backward_with_stop(speed: float) -> int:
    """Drives rover straight backward at specified speed for unspecified distance
    Input [y/yes] for it to stop driving.

    Args:
        speed (float): velocity (m/s)

    Returns:
        int: 0 no matter what
    """
    speed = float(speed)

    print(f"Driving backward at {speed:.4f} m/s")

    for wheel in WHEELS:
        wheel.set_motor_speed("backward", speed)

    # instead of waiting for specific input, easier for user to just press Enter
    # input will block until Enter pressed anyway
    input("Press Enter to stop!")

    stop_all_wheels()
    return 0


def set_arc_wheels(direction: str, radius: float) -> None:
    """Turns corner wheels to position to make arc turn

    Args:
        direction (str): Turn which way, needs to be 'right' or 'left'
        radius (float): Radius of circle, measured to center rover
    """
    recenter()
    opp_direction = "left"
    outer_deg = math.degrees(math.atan(R_HEIGHT / (radius + R_OUTER / 2)))
    inner_deg = math.degrees(math.atan(R_HEIGHT / (radius - R_OUTER / 2)))
    if opp_direction == direction:
        opp_direction = "right"
    # rotate_n_degrees doesn't check that angle > 0, so can abuse this
    # to rotate opposite of direction implied. Works well here and makes code simpler
    CORNER_FL.rotate_n_degrees(
        direction, outer_deg if direction == "right" else inner_deg
    )
    CORNER_FR.rotate_n_degrees(
        direction, outer_deg if direction == "left" else inner_deg
    )
    # rear corners turn opposite of front
    CORNER_BL.rotate_n_degrees(
        opp_direction, outer_deg if direction == "right" else inner_deg
    )
    CORNER_BR.rotate_n_degrees(
        opp_direction, outer_deg if direction == "left" else inner_deg
    )
    wait_until_all_complete()


def move_to_point(endpoint: Point, midpoint=None) -> int:
    """Moves from current position to specified position

    Args:
        endpoint (Point): Point to move to
        midpoint (Point): Point to move to in the middle of the arc.
            Defaults to None
        
    Returns:
        int: 0 for success, -1 for failure
    """
    facing = Line(Point(0, 0), Point(1, 0))
    travel = Line(Point(0, 0), endpoint)

    if not all([motor.calibrated for motor in ALL_MOTORS]):
        calibrate_all()

    if midpoint is None:
        angle = facing.angle() - travel.angle()
        dir = "right" if angle > 0 else "left"
        angle = abs(angle)
        tank_with_turn(dir, angle)
        move_distance_meters(travel.length())
    else:
        travel_path = Arc.from_points(Point(0, 0), midpoint, endpoint)
        travel = travel_path.tangent(Point(0, 0))

        angle = facing.angle() - travel.angle()
        dir = "right" if angle > 0 else "left"
        angle = abs(angle)
        tank_with_turn(dir, angle)

        dir = "right" if midpoint.arr[0] < 0 else "left"
        arc_turn_drive(dir, travel_path.radius, travel_path.length, "forward")


# drives rover forward for arc turn at specified speed for specified distance
def arc_turn_drive(direction, radius, dist, drive) -> int:
    """Process of doing an arc turn. Has constant speed determined by constants at top of file.
    Make use of Motor.move_distance function

    Args:
        direction (str): specifies which way to turn, needs to be either 'right' or 'left'
        radius (float): distance from center rover to center circle for the arc that is about to be traveled
        dist (float): arc length to traverse
        drive (str): which way to spin wheels, 'forward' or 'backward'

    Returns:
        int: 0 on successful completion, -1 if errors in parameters
    """
    out_cen_speed = MAX_SPEED_VEL
    out_speed = out_cen_speed * (
        math.sqrt(radius ** 2 + R_OUTER * radius + 0.101) / (radius + 0.155)
    )
    inner_speed, inner_cen_speed = get_inner_velo(MAX_TURN, out_cen_speed)
    timer = get_time(out_cen_speed, dist)
    if direction == "right":  # right turn: right inner, left outer
        set_speed_left_side(drive, out_cen_speed, out_speed)
        set_speed_right_side(drive, inner_cen_speed, inner_speed)
    else:  # left turn: left inner, right outer
        set_speed_left_side(drive, inner_cen_speed, inner_speed)
        set_speed_right_side(drive, out_cen_speed, out_speed)

    time.sleep(timer)

    stop_all_wheels()

    return 0


"""Not working cause GPIO is not being used"""
# def follow_me() -> None:
#     """Tries to follow something in front of it

#     Args:
#         stop (function): a function that checks if we need to stop running
#     """
#     # assume the lidar is running
#     STOPPED = 0
#     MOVING = 1
#     TURNING = 2

#     centered = True

#     if not all([corner.calibrated for corner in CORNERS]):
#         calibrate_all()
#     else:
#         recenter()

#     currentState = STOPPED

#     while follow_stop.stop is False:

#         # move forward
#         if GPIO.input(GO_PIN):
#             if currentState != MOVING:
#                 if not centered:
#                     recenter()
#                     centered = True
#                 set_speed_all("forward")
#                 currentState = MOVING

#         # stop
#         elif GPIO.input(STOP_PIN):
#             if currentState != STOPPED:
#                 stop_all_wheels()
#                 currentState = STOPPED

#         # turn left
#         elif GPIO.input(LEFT_PIN):
#             if currentState != TURNING:
#                 stop_all_wheels()
#                 tank_align()
#                 centered = False
#             set_speed_left_side("backward")
#             set_speed_right_side("forward")
#             currentState = TURNING

#         # turn right
#         elif GPIO.input(RIGHT_PIN):
#             if currentState != TURNING:
#                 stop_all_wheels()
#                 tank_align()
#                 centered = False
#             set_speed_left_side("forward")
#             set_speed_right_side("backward")
#             currentState = TURNING


# def static_vars(**kwargs):
#     def decorate(func):
#         for k in kwargs:
#             setattr(func, k, kwargs[k])
#         return func
#     return decorate

# @static_vars(stop=False)
# def follow_stop():
#     follow_stop.stop = True