"""Communicate with a Thorlabs elliptec controller."""

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

errmsg = {0: "OK, no error",
          1: "Communication timeout",
          2: "Mechanical timeout",
          3: "Command error or not supported",
          4: "Value out of range",
          5: "Module isolated",
          6: "Module out of isolation",
          7: "Initializing error",
          8: "Thermal error",
          9: "Busy",
          10: "Sensor error",
          11: "Motor error",
          12: "Out of range",
          13: "Over current error",
          14: "Reserved"}

# Direction constants
CW = 0
CCW = 0

# Acceptable accuracy
DEGERR = 0.1
MMERR = 0.05

# Device ids by module type
modtype = {"linear": [7, 10, 17, 20],
           "rotary": [8, 14, 18],
           "indexed": [6, 9],
           "optclean": [14, 17, 18, 20]}
# Linear plus rotary stages have same home cmd, same number of motors
modtype["linrot"] = modtype["linear"] + modtype["rotary"]


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

    def __init__(self,
                 dev,
                 addrs,
                 home=True,
                 freq=True,
                 freqSave=False,
                 cal=dict()):
        """Initialize communication with controller and home all modules."""
        self.openserial(dev)
        # info: module/motor info received during init
        self.info = dict()
        # zero: per-module user calibration offset
        self.zero = cal
        self.ser.timeout = 6
        self.addrs = addrs
        for addr in addrs:
            info = self.information(addr)
            if not info:
                raise MissingModule("No module found at supplied address")
            else:
                self.initinfo(addr, info)
                # Also initialize the cal offset to zero
                self.zero[addr] = 0
                # The (second) initial frequency scan's result is not
                # saved by default
                if freq:
                    self.searchfreq(addr)
                    if freqSave:
                        self.saveuserdata(addr)
                # An initial homing must be performed to establish a
                # datum for subsequent moving
                if home:
                    self.home(addr)
        self.ser.timeout = 2

    def handler(self, retval):
        """Process replies from modules.

        By wrapping messages to and from the modules, errors and other
        status information can optionally be enabled here.
        """
        return retval

    def _readmsgs(self, count):
        """Collect the expected number of replies from the modules."""
        msgs = []
        for i in range(count):
            msgs.append(self.ser.readline().decode())
        return msgs

    def clearmsgs(self):
        """Clear message backlog until serial timeout is reached."""
        retval = ''
        msgs = []
        while not retval:
            msgs.append(self.ser.readline().decode())
        return msgs

    def openserial(self, dev):
        """Open serial connection."""
        self.ser = serial.Serial(dev,
                                 9600,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=2)

    def close(self):
        """Shut down the buffer and serial connection cleanly."""
        # TODO: Still not sure how to sequentially close these without errors.
        # Should sio be closed too?
        self.ser.close()

    def bufmsg(self, msg):
        """Send message to module and wait on readline() for a response."""
        self.ser.write(msg.encode())
        retval = self.ser.readline().decode()
        return retval

    def _sndmsg(self, msg):
        """Send message to module without waiting for a response."""
        self.ser.write(msg)

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

    def isstatus(self, retval):
        """Check if retval is a status code."""
        if retval[1:3] == "GS":
            return True
        else:
            return False

    def parsestatus(self, retval):
        """Convert status retval for comparison to status constants."""
        return int(retval[3:5], 16)

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

    def searchfreq1(self, addr):
        """Scan and optimize resonant frequency of motor 1."""
        return self.handler(self.msg(addr, 's1'))

    def searchfreq2(self, addr):
        """Scan and optimize resonant frequency of motor 2."""
        return self.handler(self.msg(addr, 's2'))

    def searchfreq3(self, addr):
        """Scan and optimize resonant frequency of motor 3."""
        return self.handler(self.msg(addr, 's3'))

    def searchfreq(self, addr):
        """Scan and optimize resonant frequencies of all motors."""
        if self.info[addr]["partnumber"] in modtype["indexed"]:
            s1 = self.searchfreq1(addr)
            if s1 == MECH_TIMEOUT:
                raise ReportedError(errmsg[s1])
        elif self.info[addr]["partnumber"] in modtype["linrot"]:
            s1 = self.parsestatus(self.searchfreq1(addr))
            s2 = self.parsestatus(self.searchfreq2(addr))
            if s1 == MECH_TIMEOUT or s2 == MECH_TIMEOUT:
                raise ReportedError(errmsg[MECH_TIMEOUT])
        else:
            raise ModuleError

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

    def ispos(self, ret):
        """Check if return string is position report."""
        if ret[1:3] == "PO":
            return True
        else:
            return False

    def pos2deg(self, addr, retval):
        """Convert pos retval to degrees using queried scale factor."""
        return self.step2deg(addr, self.hex2step(retval[3:11]))

    def pos2mm(self, addr, retval):
        """Convert pos retval to mm using queried scale factor."""
        return self.step2mm(addr, self.hex2step(retval[3:11]))

    def home(self, addr, direction=CCW):
        """Move motor to home position.

        For rotary stages, byte 3 sets direction: 0 CW and 1 CCW.
        """
        if self.info[addr]["partnumber"] in modtype["linrot"]:
            if direction == CW:
                return self.handler(self.msg(addr, 'ho0'))
            else:
                return self.handler(self.msg(addr, 'ho1'))
        else:
            return self.handler(self.msg(addr, 'ho'))

    def homeall(self, direction=CCW):
        """Home all connected modules."""
        for addr in self.addrs:
            self.home(addr, direction)

    def deg2step(self, addr, deg):
        """Convert degrees to steps using queried scale factor."""
        return int(deg * self.info[addr]["pulses"]/360)

    def step2deg(self, addr, step):
        """Convert steps to degrees using queried scale factor."""
        return step * 360 / self.info[addr]["pulses"]

    def mm2step(self, addr, mm):
        """Convert mm to steps using queried scale factor."""
        return int(mm * self.info[addr]["pulses"])

    def step2mm(self, addr, step):
        """Convert steps to mm using queried scale factor."""
        return step / self.info[addr]["pulses"]

    @staticmethod
    def hex2step(x):
        """Convert hex-encoded int32 to python int."""
        if len(x) != 8:
            raise ValueError
        y = int(x, 16)
        if y > 1 << (32 - 1):
            y -= 1 << 32
        return y

    @staticmethod
    def step2hex(step):
        """Convert int32 to hex-encoded string understood by controller.

        Note that [a-f] are NOT accepted as valid hex values by the
        module controllers. To make step2hex and hex2step bijective, we
        consider negative values as well.
        """
        if step < 0:
            step += 1 << 32
        return hex(step)[2:].zfill(8).upper()

    def _moveabsolute(self, addr, pos):
        """Move motor to specified absolute position (dumb version)."""
        if self.info[addr]["partnumber"] in modtype["rotary"]:
            step = self.deg2step(addr, pos)
        elif self.info[addr]["partnumber"] in modtype["linear"]:
            step = self.mm2step(addr, pos)
        else:
            raise ModuleError
        hstep = self.step2hex(step)
        return self.handler(self.msg(addr, 'ma' + hstep))

    def moveabsolute(self, addr, pos, depth=1):
        """Move motor to specified absolute position."""
        ret = self._moveabsolute(addr, pos)
        # Command was not received, need to retry
        if ret == "":
            # Eventually give up
            if depth > 5:
                raise ReportedError("Command unsuccessful after 5 tries")
            else:
                return self.moveabsolute(addr, pos, depth=(depth + 1))
        # Check reported position and retry if not within error
        elif self.ispos(ret) and (ret[0] == addr):
            if depth > 5:
                raise ReportedError("Command unsuccessful after 5 tries")
            else:
                if self.info[addr]["partnumber"] in modtype["rotary"]:
                    if abs(self.pos2deg(addr, ret) - pos) > DEGERR:
                        return self.moveabsolute(addr, pos, depth=(depth + 1))
                    else:
                        return ret
                else:
                    if abs(self.pos2mm(addr, ret) - pos) > MMERR:
                        return self.moveabsolute(addr, pos, depth=(depth + 1))
                    else:
                        return ret
        # Pass on a reported error
        elif self.isstatus(ret):
            raise ReportedError(errmsg[self.parsestatus(ret)])
        else:
            raise Error("moveabsolute unsuccessful")

    def moverelative(self, addr, delta):
        """Move motor relative to current position."""
        if self.info[addr]["partnumber"] in modtype["rotary"]:
            step = self.deg2step(addr, delta)
        elif self.info[addr]["partnumber"] in modtype["linear"]:
            step = self.mm2step(addr, delta)
        else:
            raise ModuleError
        hstep = self.step2hex(step)
        return self.handler(self.msg(addr, 'mr' + hstep))

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
            ret = []
            for addr, x in zip(addrs, xs):
                if self.info[addr]["partnumber"] in modtype["rotary"]:
                    y = (self.zero[addr] + x) % 360
                else:
                    y = self.zero[addr] + x
                ret.append(self.moveabsolute(addr, y))
            return ret
        except TypeError:
            if self.info[addr]["partnumber"] in modtype["rotary"]:
                y = (self.zero[addr] + x) % 360
            else:
                y = self.zero[addr] + x
            self.moveabsolute(addrs, y)
