"""Communicate with a Thorlabs elliptec controller."""

import io
import serial

# Error codes for controller
OK               = 00
COMM_TIMEOUT     = 01
MECH_TIMEOUT     = 02
COMMAND_ERR      = 03
VAL_OUT_OF_RANGE = 04
MOD_ISOLATED     = 05
MOD_OUT_OF_ISOL  = 06
INIT_ERROR       = 07
THERMAL_ERROR    = 08
BUSY             = 09
SENSOR_ERROR     = 10
MOTOR_ERROR      = 11
OUT_OF_RANGE     = 12
OVER_CURRENT     = 13
GENERAL_ERROR    = 14

# Error handling modes
PASS = 0
ERROR = 1
LAZY = 2

class Error(Exception):
    """Base class"""
    pass

class ElliptecError(Error):
    """Reply to command indicates an error has occured"""
    pass

class Elliptec:
    """Initialize serial communication with the elliptec controller, and
    probe the motors at the supplied addresses to gather information and
    perform an intial homing."""
    def __init__(self, dev, addrs):
        self.openserial(dev)
        self.openbuffer()
        self.scaling = {}
        self.zero = {}
        for addr in addrs:
            info = self.ident(addr)
            # TODO: logic to determine which type of device is at each
            # address should go here. Individual methods that depend
            # on the device type should access device information
            # stored here.
            if not info:
                print('Motor ' + str(addr) + ' not found!')
            else:
                self.scaling[addr] = int(info.strip()[-8:], 16)
                # An initial homing must be performed to establish a
                # datum for subsequent moving
                self.home(addr)

    def close(self):
        """Shut down the buffer and serial connection cleanly."""
        # Still not sure how to sequentially close these without errors
        self.sio.close()
        self.ser.close()

    def openserial(self, dev):
        """Open serial connection."""
        self.ser = serial.Serial(dev,
                                 9600,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=1)

    def openbuffer(self):
        """Open io buffer wrapping serial connection."""
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser),
                                    newline='\r\n')

    def bufmsg(self, msg):
        """Primitive to send message to controller and return response."""
        self.sio.write(msg)
        self.sio.flush()
        return self.sio.readline()

    def msg(self, addr, msg):
        """Convenience function for commands which begin with address."""
        addr = str(int(addr, 16))
        return self.bufmsg(addr + msg)

    def ident(self, addr):
        """Get basic identification from motor.
        Can be used to check if motor is connected."""
        return self.msg(addr, 'in')

    def motorinfo(self, addr):
        """More detailed motor information."""
        i = self.msg(addr, 'i1')
        j = self.msg(addr, 'i2')
        return i + j

    def changeaddress(self, addr, naddr):
        """Change address of module at addr to naddr."""
        return self.msg(addr, 'ca' + naddr)

    def groupaddress(self, addr, gaddr):
        """Instruct module at addr to respond to the next command sent
        to gaddr, in order to operate simultaneously. It will switch
        back to responding to its own address after one operation.
        Responses from multiple devices are ordered by their address,
        with 0x0 having priority."""
        return self.msg(addr, 'ga' + gaddr)

    def cleanmechanics(self, addr):
        """Perform cleaning cycle on module. Note: takes several
        minutes and will block other commands, replying with busy.
        Other modules on the same bus may have performance affected
        during this time."""
        return self.msg(addr, 'cm')

    def homeoff(self, addr):
        """Requests the motor's home position, relative to the absolute
        limit of travel.
        Note: Thorlabs warns NOT to set the home position to a custom
              value, since it is factory-set to ensure consistent
              homing. For the ELL14 rotation mount, the position can
              only be changed up to 90 deg. See the setcal/cal* methods
              for a convenient way to set an arbitrary offset for
              successive movements."""
        return self.msg(addr, 'go')

    def jogstep(self, addr):
        """Requests jog length."""
        return self.msg(addr, 'gj')

    def pos(self, addr):
        """Requests current motor position."""
        return self.msg(addr, 'gp')

    def home(self, addr):
        """Move motor to home position. For rotary stages, byte 3 sets
        direction: 0 CW and 1 CCW."""
        return self.msg(addr, 'ho1')

    def deg2step(self, addr, deg):
        """Use scaling factor queried from motor during init."""
        return int(deg * self.scaling[addr]/360)

    @staticmethod
    def step2hex(step):
        """Controller accepts hex postions, note that [a-f] are NOT
        accepted as valed hex values."""
        return hex(step)[2:].zfill(8).upper()

    def absm(self, addr, deg):
        """Move motor to specified absolute position."""
        step = self.deg2step(addr, deg)
        hstep = self.step2hex(step)
        return self.msg(addr, 'ma' + hstep)

    def setcal(self, addrs, angles):
        """Set a calibration offset for motor at addr, so that the cal-
        series of commands will move relative to this offset from the
        home position. Accepts an individual address and angle, or lists
        of both."""
        try:
            for addr, angle in zip(addrs, angles):
                self.zero[addr] = angle
        except TypeError:
            self.zero[addrs] = angles

    def calmove(self, addrs, angles):
        """Move motor at addr to angle relative to calibration offset.
        Accepts an individual address and angle, or lists of both."""
        try:
            for addr, angle in zip(addrs, angles):
                self.absm(addr, self.zero[addr] + angle)
        except TypeError:
            self.absm(addrs, self.zero[addrs] + angles)
