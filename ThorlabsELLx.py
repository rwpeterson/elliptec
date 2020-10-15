import io
import serial


class ThorlabsELLx:
    def __init__(self):
        pass
        

    def elliptec_open(self,dev):
        ser = serial.Serial(dev,
                            9600,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=1)
        return ser

    def elliptec_buffer(self,ser):
        sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser),
                            newline='\r\n')
        return sio

    def bufmsg(self,sio, msg):
        sio.write(msg)
        sio.flush()
        return sio.readline()

    def emsg(self,sio, addr, msg):
        addr = str(int(addr, 16))
        return self.bufmsg(sio, addr + msg)

    def eident(self,sio, addr):
        return self.emsg(sio, addr,'in')

    def emotorinfo(self,sio, addr):
        i1 = self.emsg(sio, addr, 'i1')
        i2 = self.emsg(sio, addr, 'i2')
        return i1 + i2

    def ehomeoff(self,sio, addr):
        return self.emsg(sio, addr, 'go')

    def ejogstep(self,sio, addr):
        return self.emsg(sio, addr, 'gj')

    def epos(self,sio, addr):
        return self.emsg(sio, addr, 'gp')

    def ehome(self,sio, addr):
        return self.emsg(sio, addr, 'ho1')

    def deg2estep(self,deg):
        return int(deg * 262144/360)

    def estep2ehex(self,step):
        return hex(step)[2:].zfill(8).upper()

    def eabsm(self, sio, addr, deg):
        step = self.deg2estep(deg)
        hstep = self.estep2ehex(step)
        return self.emsg(sio, addr, 'ma' + hstep)
