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

    def getinfo(self, addr):
        """Get information about module.
        
        Reply format:
        | 0 | 1-2  | 3-4 | 5-12 | 13-16 | 17-18 | 19-20 | 21-24  | 25-32  |
        | A | "IN" | ELL | SN   | YEAR  | FWREL | HWREL | TRAVEL | PULSES |
        
        Definitions:
            A      - address
            "IN"   - command name
            ELL    - ELLxx part number
            SN     - Serial number
            YEAR   - year of mfg
            FWREL  - firmware release
            HWREL  - hardware release:
                     MSB is thread (1 imperial/0 metric)
                     rest is 7-bit hardware release
            TRAVEL - travel in mm/deg
            PULSES - pulses per position (bi-positional only)
        """
        return self.handler(self.msg(addr, 'in'))

    def initinfo(self, addr, info):
        """Parse and store module information string."""
        self.info[addr] = {}
        self.info[addr]["partnumber"] = info.strip()[3:5]
        self.info[addr]["serialnumber"] = info.strip()[5:13]
        self.info[addr]["year"] = info.strip()[13:17]
        self.info[addr]["fwrel"] = info.strip()[17:19]
        self.info[addr]["hwrel"] = info.strip()[19:21]
        self.info[addr]["travel"] = info.strip()[21:25]
        self.info[addr]["pulses"] = int(info.strip()[25:33]
        return

    def motorinfo(self, addr):
        """More detailed motor information."""
        i = self.msg(addr, 'i1')
        j = self.msg(addr, 'i2')
        return self.handler(i + j)

    def getmotorinfo1(self, addr):
        """Get motor 1 parameters from module.

        Reply format:
        | 0 | 1-2  | 3    | 4     | 5-8     | 9-12   | 13-16  | 17-20  | 21-24  |
        | A | "I1" | LOOP | MOTOR | CURRENT | RAMPUP | RAMPDN | FWDPER | BAKPER |

        Definitions:
        A - address
        "I1" - command name
        LOOP - state of loop (1 = ON)
        MOTOR - state of motor (1 = ON)
        CURRENT - 1866 points is 1 amp
        RAMPUP - PWM increase every ms (0xFFFF undef.)
        RAMPDN - PWN decrease every ms
        FWDPER - forward period value
        BAKPER - backward period value
        period value - 14,740,000/frequency
        """
        return self.handler(self.msg(addr, 'i1'))

    def getmotorinfo2(self, addr):
        """Get motor 2 parameters from module, for devices which have two motors."""
        return self.handler(self.msg(addr, 'i2'))

    def storemotorinfo(self, num, m):
        """Parses and stores motor info for motor <num>."""
        self.info[addr][num] = {}
        self.info[addr][num]["loop"]           = int(m.strip()[3:4])
        self.info[addr][num]["motor"]          = int(m.strip()[4:5])
        self.info[addr][num]["current"]        = int(m.strip()[5:9], 16) / 1866
        self.info[addr][num]["rampup"]         = int(m.strip()[9:13], 16)
        self.info[addr][num]["rampdown"]       = int(m.strip()[13:17], 16)
        self.info[addr][num]["forwardperiod"]  = 14740000 / int(m.strip()[17:21], 16)
        self.info[addr][num]["backwardperiod"] = 14740000 / int(m.strip()[21:25], 16)
    
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
