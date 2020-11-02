"""Communicate with a Thorlabs elliptec controller."""

import io
import serial

# Error codes for controller
OK = 0
COMM_TIMEOUT = 1
MECH_TIMEOUT = 2
COMMAND_ERR = 3
VAL_OUT_OF_RANGE = 4
MOD_ISOLATED = 5
MOD_OUT_OF_ISOL = 6
INIT_ERROR = 7
THERMAL_ERROR = 8
BUSY = 9
SENSOR_ERROR = 10
MOTOR_ERROR = 11
OUT_OF_RANGE = 12
OVER_CURRENT = 13
GENERAL_ERROR = 14

# Error handling modes
PASS = 0
ERROR = 1
LAZY = 2

# Direction constants
CW = 0
CCW = 0

class Error(Exception):
    """Base class."""

    pass


class ElliptecError(Error):
    """Reply to command indicates an error has occured."""

    pass


class Elliptec:
    """Create class to manage Elliptec controller and connected modules.

    Class initializes serial communication with the elliptec controller, and
    probes the modules at the supplied addresses to gather information and
    perform an intial homing.
    """

    def __init__(self, dev, addrs, hmode=PASS):
        self.openserial(dev)
        self.openbuffer()
        self.info = dict()
        self.zero = dict()
        self.hmode = PASS
        self.hct = 0
        self.sct = 0
        self.ser.timeout = 2
        for addr in addrs:
            info = self.getinfo(addr)
            self.initinfo(addr, info)
            # TODO: logic to determine which type of device is at each
            # address should go here. Individual methods that depend
            # on the device type should access device information
            # stored here.
            if not info:
                print('Motor ' + str(addr) + ' not found!')
            else:
                # An initial homing must be performed to establish a
                # datum for subsequent moving
                self.home(addr)
        self.ser.timeout = 1
        self.hmode = hmode

    def handler(self, retval):
        """Process replies from modules according to error mode.

        By setting hmode in the Elliptec() class, handle errors returned
        vis the modules accoring to the following modes.

        Modes
        -----
        PASS
            After every command, wait on readline() to capture one
            line of reply. Then pass it without checking for an error. This
            is a crude solution for commands which always result in one
            reply, blocking until that reply is received. It will cause
            problems if multiple lines are received per command. Examples
            include errors encountered and reported autonomously by the
            modules, and the result of commands like groupaddress that cause
            multiple modules to obey (and respond) to a single command.
        LAZY
            Send commands, keep track of the expected number of
            replies but wait until a user-defined time to process them
            (and possibly check for additional errors)
        """
        if self.hmode == PASS:
            return retval
        elif self.hmode == LAZY:
            return

    def readmsgs(self):
        """Collect the expected number of replies from the modules."""
        msgs = []
        self.sio.flush()
        for i in range(self.sct):
            msgs.append(self.sio.readline())
        self.sct = 0
        return msgs

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

    def close(self):
        """Shut down the buffer and serial connection cleanly."""
        # TODO: Still not sure how to sequentially close these without errors.
        # Should sio be closed too?
        self.ser.close()

    def bufmsg(self, msg):
        """Send message to module and wait on readline() for a response."""
        self.sio.write(msg)
        self.sio.flush()
        return self.sio.readline()

    def sndmsg(self, msg):
        """Send message to module without waiting for a response."""
        self.sio.write(msg)
        self.sio.flush()

    def msg(self, addr, msg):
        """Send message to module, handling reply according to hmode."""
        addr = str(int(addr, 16))
        if self.hmode == PASS or self.hmode == ERROR:
            return self.bufmsg(addr + msg)
        elif self.hmode == LAZY:
            self.sct += 1
            self.sndmsg(addr + msg)

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
        self.info[addr]["partnumber"] = int(info.strip()[3:5], 16)
        self.info[addr]["serialnumber"] = int(info.strip()[5:13])
        self.info[addr]["year"] = int(info.strip()[13:17])
        self.info[addr]["fwrel"] = int(info.strip()[17:19])
        self.info[addr]["hwrel"] = int(info.strip()[19:21])
        self.info[addr]["travel"] = int(info.strip()[21:25], 16)
        self.info[addr]["pulses"] = int(info.strip()[25:33], 16)

    def getmotorinfo1(self, addr):
        """Get motor 1 parameters from module.

        Reply format:
        | 0 | 1-2 | 3    | 4     | 5-8   | 9-12   | 13-16  | 17-20  | 21-24  |
        | A | I1  | LOOP | MOTOR | CRRNT | RAMPUP | RAMPDN | FWDPER | BAKPER |

        Definitions:
        A - address
        "I1" - command name
        LOOP - state of loop (1 = ON)
        MOTOR - state of motor (1 = ON)
        CRRNT - 1866 points is 1 amp
        RAMPUP - PWM increase every ms (0xFFFF undef.)
        RAMPDN - PWN decrease every ms
        FWDPER - forward period value
        BAKPER - backward period value
        period value - 14,740,000/frequency
        """
        return self.handler(self.msg(addr, 'i1'))

    def getmotorinfo2(self, addr):
        """Get motor 2 parameters from module.

        Only applies for devices which have two motors.
        """
        return self.handler(self.msg(addr, 'i2'))

    def storemotorinfo(self, addr, num, m):
        """Parse and store motor info for motor `num`."""
        self.info[addr][num] = dict()
        self.info[addr][num]["loop"] = int(m.strip()[3:4])
        self.info[addr][num]["motor"] = int(m.strip()[4:5])
        self.info[addr][num]["current"] = int(m.strip()[5:9], 16) / 1866
        self.info[addr][num]["rampup"] = int(m.strip()[9:13], 16)
        self.info[addr][num]["rampdown"] = int(m.strip()[13:17], 16)
        self.info[addr][num]["forwardperiod"] = \
            14740000 / int(m.strip()[17:21], 16)
        self.info[addr][num]["backwardperiod"] = \
            14740000 / int(m.strip()[21:25], 16)

    def changeaddress(self, addr, naddr):
        """Change address of module at addr to naddr.

        Expected reply: GS00 from new address.
        """
        return self.handler(self.msg(addr, 'ca' + naddr))

    def saveuserdata(self, addr):
        """Instruct device to save motor parameters.

        Required for persistent custom frequencies or addresses.
        """
        return self.handler(self.msg(addr, 'us'))

    def groupaddress(self, addr, gaddr):
        """Instruct module at `addr` to respond to `gaddr`.

        The next command sent to `gaddr` will be obeyed by the module at
        `addr`, in order to operate simultaneously. It will switch back
        to responding to its own address after one operation. Responses
        from multiple devices are ordered by their address, with 0x0
        having priority.
        """
        return self.handler(self.msg(addr, 'ga' + gaddr))

    def cleanmechanics(self, addr):
        """Perform cleaning cycle on module.

        Note: takes several minutes and will block other commands,
              replying with busy. Other modules on the same bus may have
              performance affected during this time.
        """
        return self.handler(self.msg(addr, 'cm'))

    def homeoffset(self, addr):
        """Request the motor's home position.

        Note: Thorlabs warns NOT to set the home position to a custom
              value, since it is factory-set to ensure consistent
              homing. For the ELL14 rotation mount, the position can
              only be changed up to 90 deg. See the setcal/cal* methods
              for a convenient way to set an arbitrary offset for
              successive movements.
        """
        return self.handler(self.msg(addr, 'go'))

    def jogstep(self, addr):
        """Request jog length."""
        return self.handler(self.msg(addr, 'gj'))

    def pos(self, addr):
        """Request current motor position."""
        return self.handler(self.msg(addr, 'gp'))

    def home(self, addr, dir=CCW):
        """Move motor to home position.

        For rotary stages, byte 3 sets direction: 0 CW and 1 CCW.
        """
        if self.info[addr]["partnumber"] == 14:
            if dir == CW:
                return self.handler(self.msg(addr, 'ho0'))
            else:
                return self.handler(self.msg(addr, 'ho1'))
        else:
            return self.handler(self.msg(addr, 'ho'))

    def deg2step(self, addr, deg):
        """Use scaling factor queried from motor during init."""
        return int(deg * self.info[addr]["pulses"]/360)

    @staticmethod
    def step2hex(step):
        """Convert int to hex-encoded string understood by controller.

        Note that [a-f] are NOT accepted as valed hex values.
        """
        return hex(step)[2:].zfill(8).upper()

    def moveabsolute(self, addr, deg):
        """Move motor to specified absolute position."""
        step = self.deg2step(addr, deg)
        hstep = self.step2hex(step)
        return self.handler(self.msg(addr, 'ma' + hstep))

    def setcal(self, addrs, angles):
        """Set a calibration offset for motor at `addr`.

        The cal- series of commands will move relative to this offset from the
        home position. Accepts an individual address and angle, or lists
        of both.
        """
        try:
            for addr, angle in zip(addrs, angles):
                self.zero[addr] = angle
        except TypeError:
            self.zero[addrs] = angles

    def calmove(self, addrs, angles):
        """Move motor at addr to angle relative to calibration offset.

        Accepts an individual address and angle, or lists of both.
        """
        try:
            for addr, angle in zip(addrs, angles):
                self.moveabsolute(addr, self.zero[addr] + angle)
        except TypeError:
            self.moveabsolute(addrs, self.zero[addrs] + angles)
