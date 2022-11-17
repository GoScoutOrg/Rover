import i2c_bus as i2c
import struct
import RPi.GPIO as GPIO
from .util import PowerGPIO

'''
Commands are run in try/except blocks in order to catch remote IO errors.

These errors can occur if the I2C connection is shoddy, or if there is some
    weird bus issue. 

If its a bus issue, I restarted both the Pi and the Jetson and that seems to
    resolve it 90% of the time. This can likely be cut down to only restarting
    the Jetson. If issues persist the system should assume the Jetson is out of
    commission.
'''

class JetsonError(Exception):
    pass

class Jetson:
    '''
    Jetson class that holds commands for interacting with the Jetson Nano, 
        providing an interface that abstracts away all the i2c shenanigans.
    '''

    def __init__(self, vision = False):
        '''
        Initializes the I2C bus for communication (?)
        '''
        self.bus = i2c.I2CBus()
        self.vision = vision

        if vision:
            try:
                self.vision_start()

            except JetsonError:
                print('help me')
                self.vision = False

    def __del__(self):
        if self.vision:
            self.vision_stop()

    def vision_start(self):
        '''Starts the vision pipe, needs an i2c_bus object to send command.'''
        try:
            if self.bus.send_and_wait('vision start'.encode(), 'c', 0):
                return self
            else:
                raise JetsonError('Error starting vision pipe')
        except OSError:
            raise JetsonError('Error starting vision pipe - OSError')

        self.vision = True

    def vision_stop(self):
        '''Stops the vision pipe'''
        try:
            if not self.bus.send_and_wait('vision stop'.encode(), 'c', 0):
                raise JetsonError('Error stopping vision pipe')
        except OSError:
            raise JetsonError('Error stopping vision pipe - OSError')

            
class Jetson:
    '''
    Jetson class that holds commands for interacting with the Jetson Nano, providing an interface that 
        abstracts away all the i2c shenanigans.
    '''

    def __init__(self):
        '''
        Initializes the I2C bus for communication (?)
        '''
        self.bus = i2c.I2CBus()
        self.enable = PowerGPIO.Jetson # GPIO 16

        GPIO.setup(self.enable, GPIO.OUT)

        self.vision = False

    def power_on(self):
        '''
        Uses GPIO to turn power to Jetson on
        '''
        GPIO.output(self.enable, GPIO.HIGH)

    def power_off(self):
        '''
        Uses GPIO to turn power to Jetson off
        '''
        GPIO.output(self.enable, GPIO.LOW)

    def power_mode_high(self):
        '''
        Puts the Jetson into normal full power mode:
            4 cores
            X GHz

        Returns True if command properly actioned

        Returns False if there was an error (timeout or pylibi2c write error)
        '''

        try:
            if self.bus.send_and_wait('power high'.encode(), 'c', 0):
                return True
            else:
                return False

        # Catch remote IO errors
        except OSError:
            return False

    def power_mode_low(self):
        '''
        Puts the Jetson into low power mode for less power consumption:
            1 core
            X GHz

        Returns True if command properly actioned

        Returns False if there was an error (timeout or pylibi2c write error)
        '''

        try:
            if self.bus.send_and_wait('power low'.encode(), 'c', 0):
                return True
            else:
                return False

        # Catch remote IO errors
        except OSError:
            return False

    def get_object(self):
        '''
        **Requires Vision Pipe**

        Requests Jetson to give object data, returns a tuple containing object 
        information:
            bool:  If there is an object in front of us
            float: How far in front of us the object is in meters
            char:  String saying where object is

        Returns tuple if successful
            Considered this returning the distance if it detected an object, 
            however I'm unsure how we would be able to distinguish between 
            "there is no object" and "there was an error in sending the packet"

        Returns False if there was an error (timeout or pylibi2c write error)
        '''

        format_str = '?f6s'
        
        # Ensure we are running this command with the vision pipe active
        if self.vision:
            try:
                # Get object information
                pkt = self.bus.send_and_wait('get object'.encode(), 'c', 0)

                if pkt:
                    # Grab data and its length
                    data = pkt[i2c.I2CPacket.data_index]
                    data_len = pkt[i2c.I2CPacket.dlen_index]

                    # Check that the data length matches the struct size
                    if data_len == struct.calcsize(format_str):
                        # Shorten data to its length
                        data = data[:data_len]

                        # Return the tuple
                        return struct.unpack(format_str, data)

                    else:
                        raise JetsonError('Received data length does not match struct length')

                else:
                    raise JetsonError('Get object timeout')

            # Catch remote IO errors
            except OSError:
                raise JetsonError('Get object OSError')

    def get_rgb_image(self):
        '''
        **Requires Vision Pipe**

        Requests Jetson to take a picture to send to the Pi

        Returns True if successful

        Returns False if there was an error (timeout or pylibi2c write error)
        '''

        # Ensure we are running this command with the vision pipe active
        if self.vision:
            try:
                if self.bus.send_and_wait('take picture'.encode(), 'c', 0):
                    # Picture taken
                    if self.bus.read_file('rgb.jpeg'):
                        # File transferred
                        return True
        
                # Picture not taken or error in file transfer.
                return False

            # Catch remote IO errors
            except OSError:
                return False

    def get_depth_image(self):
        '''
        **Requires Vision Pipe**

        Requests Jetson to take a depth picture to send to the Pi

        Returns True if successful

        Returns False if there was an error (timeout or pylibi2c write error)
        '''

        # Ensure we are running this command with the vision pipe active
        if self.vision:
            try:
                if self.bus.send_and_wait('take depth image'.encode(), 'c', 0):
                    # Picture taken
                    if self.bus.read_file('depth.jpeg'):
                        # File transferred
                        return True
        
                # Picture not taken or error in file transfer.
                return False

            # Catch remote IO errors
            except OSError:
                return False
