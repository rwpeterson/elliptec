"""Communicate with a Thorlabs elliptec controller."""

import io
import serial

# Error codes for controller
OK               = 0x00
COMM_TIMEOUT     = 0x01
MECH_TIMEOUT     = 0x02
COMMAND_ERR      = 0x03
VAL_OUT_OF_RANGE = 0x04
MOD_ISOLATED     = 0x05
MOD_OUT_OF_ISOL  = 0x06
INIT_ERROR       = 0x07
THERMAL_ERROR    = 0x08
BUSY             = 0x09
SENSOR_ERROR     = 0x10
MOTOR_ERROR      = 0x11
OUT_OF_RANGE     = 0x12
OVER_CURRENT     = 0x13
GENERAL_ERROR    = 0x14

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
    probe the modules at the supplied addresses to gather information and
    perform an intial homing."""
    def __init__(self, dev, addrs, hmode=PASS):
        self.openserial(dev)
        self.openbuffer()
        self.info = {}
        self.zero = {}
        self.hmode = hmode
        self.hct = 0
        self.sct = 0
        self.ser.timeout=2
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
        self.ser.timeout=1

    def handler(self, retval, statusbusy, statusfinal):
        """Processes replies from modules, optionally handling errors
        and deferring resolution of busy modules until later.
        
        ## Modes

        PASS: After every command, wait on readline() to capture one
        line of reply. Then pass it without checking for an error. This
        is a crude solution for commands which always result in one
        reply, blocking until that reply is received. It will cause
        problems if multiple lines are received per command. Examples
        include errors encountered and reported autonomously by the
        modules, and the result of commands like groupaddress that cause
        multiple modules to obey (and respond) to a single command.

        ERROR: Same as PASS, but raise an error if the reply is
        not as expected.

        LAZY: Send commands, keep track of the expected number of
        replies but wait until a user-defined time to process them
        (and possibly check for additional errors)

        
        
        """
        if self.hmode == PASS:
            return retval
        elif self.hmode == ERROR:
            status = self.parsestatus(retval)
            if status != statusbusy or status != statusfinal:
                raise ElliptecError
            else:
                return retval
        elif self.hmode == LAZY:
            return 

    def readmsgs(self):
        """Collect the expected number of replies from the modules."""
        msgs = []
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
        # TODO: Still not sure how to sequentially close these without errors
        self.sio.close()
        self.ser.close()

    def bufmsg(self, msg):
        """Send message to module and wait on readline() for a response."""
        self.sio.write(msg)
        self.sio.flush()
        return self.sio.readline()

    def sndmsg(self, msg):
        """Send message to module without waiting for a response."""
        self.sio.write(msg)

    def msg(self, addr, msg):
        """Send message to module, handling reply according to hmode"""
        addr = str(int(addr, 16))
        if self.hmode == PASS or self.hmode == ERROR:
            return self.bufmsg(addr + msg)
        elif self.hmode == LAZY:
            self.smsg += 1
            return self.sndmsg(addr + msg)

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
        self.info[addr]["pulses"] = int(info.strip()[25:33])
        return

    def motorinfo(self, addr):
        """more detailed motor information."""
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
        """Change address of module at addr to naddr. Expected reply: GS00 from new address."""
        return self.handler(self.msg(addr, 'ca' + naddr), )
    
    def saveuserdata(self, addr):
        """Instruct device to save motor parameters (e.g. frequencies or address)."""
        return self.handler(self.msg(addr, 'us'))

    def groupaddress(self, addr, gaddr):
        """instruct module at addr to respond to the next command sent
        to gaddr, in order to operate simultaneously. it will switch
        back to responding to its own address after one operation.
        Responses from multiple devices are ordered by their address,
        with 0x0 having priority."""
        return self.handler(self.msg(addr, 'ga' + gaddr))

    def cleanmechanics(self, addr):
        """Perform cleaning cycle on module. Note: takes several
        minutes and will block other commands, replying with busy.
        Other modules on the same bus may have performance affected
        during this time."""
        return self.handler(self.msg(addr, 'cm'))

    def homeoff(self, addr):
        """Requests the motor's home position, relative to the absolute
        limit of travel.
        Note: Thorlabs warns NOT to set the home position to a custom
              value, since it is factory-set to ensure consistent
              homing. For the ELL14 rotation mount, the position can
              only be changed up to 90 deg. See the setcal/cal* methods
              for a convenient way to set an arbitrary offset for
              successive movements."""
        return self.handler(self.msg(addr, 'go'))

    def jogstep(self, addr):
        """Requests jog length."""
        return self.handler(self.msg(addr, 'gj'))

    def pos(self, addr):
        """Requests current motor position."""
        return self.handler(self.msg(addr, 'gp'))

    def home(self, addr):
        """Move motor to home position. For rotary stages, byte 3 sets
        direction: 0 CW and 1 CCW."""
        return self.handler(self.msg(addr, 'ho1'))

    def deg2step(self, addr, deg):
        """Use scaling factor queried from motor during init."""
        return int(deg * self.info[addr]["pulses"]/360)

    @staticmethod
    def step2hex(step):
        """Controller accepts hex postions, note that [a-f] are NOT
        accepted as valed hex values."""
        return hex(step)[2:].zfill(8).upper()

    def absm(self, addr, deg):
        """Move motor to specified absolute position."""
        step = self.deg2step(addr, deg)
        hstep = self.step2hex(step)
        return self.handler(self.msg(addr, 'ma' + hstep))

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
