import io
import serial


class ThorlabsELLx:
    def __init__(self, dev, addrs):
        self.ser = self.openserial(dev)
        self.sio = self.openbuffer(self.ser)
        self.scaling = {}
        for addr in addrs:
            id = self.ident(addr)
            if not id:
                print('Motor ' + str(addr) + ' not found!')
            else:
                self.scaling[addr] = int(info.strip()[-8:],16)

    def close(self):
        # Still not sure how to sequentially close these without errors
        self.sio.close()
        self.ser.close()

    def openserial(self, dev):
        ser = serial.Serial(dev,
                            9600,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=1)
        return ser

    def openbuffer(self, ser):
        sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser),
                            newline='\r\n')
        return sio

    def bufmsg(self, msg):
        self.sio.write(msg)
        self.sio.flush()
        return self.sio.readline()

    def msg(self, addr, msg):
        addr = str(int(addr, 16))
        return self.bufmsg(self.sio, addr + msg)

    def ident(self, addr):
        return self.msg(addr, 'in')

    def motorinfo(self, addr):
        i1 = self.msg(addr, 'i1')
        i2 = self.msg(addr, 'i2')
        return i1 + i2

    def homeoff(self, addr):
        return self.msg(addr, 'go')

    def jogstep(self, addr):
        return self.msg(addr, 'gj')

    def pos(self, addr):
        return self.msg(addr, 'gp')

    def home(self, addr):
        return self.msg(addr, 'ho1')

    def deg2step(self, addr, deg):
        return int(deg * self.scaling[addr]/360)

    def step2hex(self, step):
        return hex(step)[2:].zfill(8).upper()

    def absm(self, addr, deg):
        step = self.deg2step(addr, deg)
        hstep = self.step2hex(step)
        return self.msg(addr, 'ma' + hstep)
