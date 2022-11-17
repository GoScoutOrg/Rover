import os
from roboclaw_3 import Roboclaw
from util import RC_ADDR
from util import PowerGPIO, opposite_dir, UARTException, MotorPos, UART, RC_ADDR
import RPi.GPIO as GPIO
from time import sleep


def checkUart():
    printHeading("UART")


    con1 = Roboclaw("/dev/ttyS0", 115200, PowerGPIO.ML_MR)
    con2 = Roboclaw("/dev/ttyAMA1", 115200, PowerGPIO.FL_BR)
    con3 = Roboclaw("/dev/ttyAMA2", 115200, PowerGPIO.FR_BL)

    UART1 = con1.Open() != 0
    UART2 = con2.Open() != 0
    UART3 = con3.Open() != 0
    printStatusBin("ttyS0   (MRML)", UART1)
    printStatusBin("ttyAMA1 (FRBL)", UART2)
    printStatusBin("ttyAMA2 (BRFL)", UART3)

    printHeading("Hardware")

    printStatusBin("FrontRight", con3.ReadVersion(RC_ADDR.FR)[0] == 1)
    printStatusBin("FrontLeft", con2.ReadVersion(RC_ADDR.FL)[0]==1)
    printStatusBin("Middle", con1.ReadVersion(RC_ADDR.MID)[0]==1)
    printStatusBin("RearRight", con2.ReadVersion(RC_ADDR.BR)[0]==1)
    printStatusBin("RearLeft", con3.ReadVersion(RC_ADDR.BL)[0]==1)



class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

os.system('cls' if os.name == 'nt' else 'clear')

def printStatusBin(name, status):
    color = bcolors.OKGREEN if status else bcolors.FAIL
    result = "PASS" if status else "FAIL"
    spacing = "\t\t\t\t" if len(name)<2 else "\t\t\t" if len(name)<7 else "\t\t" if len(name)<15 else "\t" if len(name)<18 else ""
    print(f"{bcolors.ENDC}"
          f"{name}:{spacing}["
          f"{color}"
          f"{result}"
          f"{bcolors.ENDC}"
          f"]")

def printHeading(name):
    spacing = "\t\t" if len(name) < 7 else "\t" if len(name) < 13 else ""
    print(f"{bcolors.UNDERLINE}"
          f"\n{name}\t\t{spacing}"
          f"{bcolors.ENDC}")

print(f"{bcolors.BOLD}"
      f"\nBIT RESULTS:"
      f"{bcolors.ENDC}\n")

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BOARD)
printHeading("Connectivity")

printStatusBin("WAP", True)
printStatusBin("EthCon", False)
printStatusBin("IP", True)
printStatusBin("Telemetry", True)

checkUart()

