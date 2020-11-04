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

# Direction constants
CW = 0
CCW = 0

# Device ids by module type
modtype = {"linear": [7, 10, 17, 20],
           "rotary": [8, 14, 18],
           "indexed": [6, 9],
           "optclean": [14, 17, 18, 20]}
# Linear stages also need direction bit in home
modtype["home"] = modtype["linear"] + modtype["rotary"]


class Error(Exception):
    """Base class."""

    pass


class ReportedError(Error):
    """Reply to command indicates an error has occured."""

    pass


class ModuleError(Error):
    """The specified command is not supported for this module."""

    pass


class MissingModule(Error):
    """A module at the specified address was not found."""

    pass


class Elliptec:
    """Create class to manage Elliptec controller and connected modules.

    Class initializes serial communication with the elliptec controller, and
    probes the modules at the supplied addresses to gather information and
    perform an intial homing.
    """

    def __init__(self, dev, addrs, home=True):
        """Initialize communication with controller and home all modules."""
        self.openserial(dev)
        self.openbuffer()
        # info: module/motor info received during init
        self.info = dict()
        # zero: per-module user calibration offset
        self.zero = dict()
        self.ser.timeout = 2
        self.addrs = addrs
        for addr in addrs:
            info = self.information(addr)
            if not info:
                raise MissingModule("No module found at supplied address")
            else:
                self.initinfo(addr, info)
                # Also initialize the cal offset to zero
                self.zero[addr] = 0
                # An initial homing must be performed to establish a
                # datum for subsequent moving
                if home:
                    self.home(addr)
        self.ser.timeout = 1

    def handler(self, retval):
        """Process replies from modules.

        By wrapping messages to and from the modules, errors and other
        status information can optionally be enabled here.
        """
        return retval

    def _readmsgs(self, count):
        """Collect the expected number of replies from the modules."""
        msgs = []
        self.sio.flush()
        for i in range(count):
            msgs.append(self.sio.readline())
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

    def _sndmsg(self, msg):
        """Send message to module without waiting for a response."""
        self.sio.write(msg)
        self.sio.flush()

    def msg(self, addr, msg):
        """Send message to module, handling reply according to hmode."""
        addr = str(int(addr, 16))
        return self.bufmsg(addr + msg)

    def information(self, addr):
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

    def motor1info(self, addr):
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

    def motor2info(self, addr):
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

    def status(self, addr):
        """Get module status/error value and clear error."""
        return self.handler(self.msg(addr, 'gs'))

    def parsestatus(self, status):
        """Convert return string for comparison to status constants."""
        return int(status[3:5])

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
        """Perform cleaning cycle on module (blocking).

        Note: takes several minutes and will block other commands,
              replying with busy. Other modules on the same bus may have
              performance affected during this time.
        """
        if self.info[addr]["partnumber"] in modtype["optclean"]:
            oto = self.ser.timeout
            self.ser.timeout = 0
            retval = self.msg(addr, 'cm')
            self.ser.timeout = oto
            return self.handler(retval)
        else:
            raise ModuleError("Command not supported for this module")

    def optimizemotors(self, addr):
        """Fine-tune frequency of motor and clean track (blocking).

        Note: takes several minutes and will block other commands,
              replying with busy. Other modules on the same bus may have
              performance affected during this time.
        """
        if self.info[addr]["partnumber"] in modtype["optclean"]:
            oto = self.ser.timeout
            self.ser.timeout = 0
            retval = self.msg(addr, 'om')
            self.ser.timeout = oto
            return self.handler(retval)
        else:
            raise ModuleError("Command not supported for this module")

    def stop(self, addr):
        """Stop the optimization or cleaning process.

        Note: Applies to ELL{14,17,18,20} devices only.
        """
        if self.info[addr]["partnumber"] in modtype["optclean"]:
            return self.handler(self.msg(addr, 'st'))
        else:
            raise ModuleError("Command not supported for this module")

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

    def home(self, addr, direction=CCW):
        """Move motor to home position.

        For rotary stages, byte 3 sets direction: 0 CW and 1 CCW.
        """
        if self.info[addr]["partnumber"] in modtype["home"]:
            if direction == CW:
                return self.handler(self.msg(addr, 'ho0'))
            else:
                return self.handler(self.msg(addr, 'ho1'))
        else:
            return self.handler(self.msg(addr, 'ho'))

    def deg2step(self, addr, deg):
        """Use scaling factor queried from motor during init."""
        return int(deg * self.info[addr]["pulses"]/360)

    def mm2step(self, addr, mm):
        """Use scaling factor queried from motor during init."""
        return int(mm * self.info[addr]["pulses"])

    @staticmethod
    def step2hex(step):
        """Convert int to hex-encoded string understood by controller.

        Note that [a-f] are NOT accepted as valed hex values.
        """
        return hex(step)[2:].zfill(8).upper()

    def moveabsolute(self, addr, pos):
        """Move motor to specified absolute position."""
        if self.info[addr]["partnumber"] in modtype["rotary"]:
            step = self.deg2step(addr, pos)
        elif self.info[addr]["partnumber"] in modtype["linear"]:
            step = self.mm2step(addr, pos)
        else:
            raise ModuleError
        hstep = self.step2hex(step)
        return self.handler(self.msg(addr, 'ma' + hstep))

    def setcal(self, *args):
        """Set a calibration offset for modules `addrs` to `xs`.

        The cal series of commands will move relative to this offset
        from the home position. Accepts an individual address and position,
        or lists of both. If no addresses are provided, all initialized
        addresses will be used.
        """
        if len(args) == 1:
            addrs = self.addrs
            xs = args[0]
        elif len(args) == 2:
            addrs = args[0]
            xs = args[1]
        else:
            raise TypeError("Too many arguments")
        try:
            for addr, x in zip(addrs, xs):
                self.zero[addr] = x
        except TypeError:
            self.zero[addrs] = xs

    def calmove(self, *args):
        """Move module at addr to position x relative to calibration offset.

        Accepts an individual address and position, or lists of both. If no
        addresses are provided, all initilized addresses will be used.
        """
        if len(args) == 1:
            addrs = self.addrs
            xs = args[0]
        elif len(args) == 2:
            addrs = args[0]
            xs = args[1]
        else:
            raise TypeError("Too many arguments")
        try:
            for addr, x in zip(addrs, xs):
                self.moveabsolute(addr, self.zero[addr] + x)
        except TypeError:
            self.moveabsolute(addrs, self.zero[addrs] + xs)
