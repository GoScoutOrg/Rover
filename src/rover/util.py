from roboclaw_3 import Roboclaw
from enum import IntEnum
import threading
import time

class MotorPos(IntEnum):
    """Enumeration for the different motor positions on the rover
    """
    FL = 0
    FR = 1
    BL = 2
    BR = 3
    ML = 4
    MR = 5
    C_FL = 6
    C_FR = 7
    C_BL = 8
    C_BR = 9


class UART(IntEnum):
    """Enumeration for the different UARTs on the rover
    """
    _0 = 0
    _4 = 1
    _5 = 2


class RC_ADDR(IntEnum):
    """Mapping location of the Roboclaw on the rover
    """
    # uart 4 128
    FL = 0x80
    # uart5 AMA2 129
    FR = 0x81
    # uart4 AMA1 130
    BR = 0x82
    # uart5 131
    BL = 0x83
    # uart0 132
    MID = 0x84


class PowerGPIO(IntEnum):
    Jetson = 36  # GPIO 16
    FL_BR = 35  # GPIO 19
    RAT = 38  # GPIO 20
    FR_BL = 40  # GPIO 21
    Solar = -1  # 18 (GPIO 24)
    ML_MR = -1 # (?)


def opposite_dir(dir: str):
    """Returns the opposite of the given direction

    Args:
        dir (str): left, right, forward, backward

    Returns:
        str: the opposite of the input
    """
    if dir == 'left':
        return 'right'
    elif dir == 'right':
        return 'left'
    elif dir == 'forward':
        return 'backward'
    elif dir == 'backward':
        return 'forward'
    else:
        return None


class UARTException(Exception):
    def __init__(self, uart: Roboclaw) -> None:
        super().__init__(f"{uart.comport} failed to connect.")
