import io
import serial

def elliptec_open(dev):
    ser = serial.Serial(dev,
                        9600,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1)
    return ser

def elliptec_buffer(ser):
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser),
                           newline='\r\n')
    return sio

def bufmsg(sio, msg):
    sio.write(msg)
    sio.flush()
    return sio.readline()

def emsg(sio, addr, msg):
    addr = str(int(addr, 16))
    return bufmsg(sio, addr + msg)

def eident(sio, addr):
    return emsg(sio, addr,'in')

def emotorinfo(sio, addr):
    i1 = emsg(sio, addr, 'i1')
    i2 = emsg(sio, addr, 'i2')
    return i1 + i2

def ehomeoff(sio, addr):
    return emsg(sio, addr, 'go')

def ejogstep(sio, addr):
    return emsg(sio, addr, 'gj')

def epos(sio, addr):
    return emsg(sio, addr, 'gp')

def ehome(sio, addr):
    return emsg(sio, addr, 'ho1')

def deg2estep(deg):
    return int(deg * 262144/360)

def estep2ehex(step):
    return hex(step)[2:].zfill(8).upper()

def eabsm(sio, addr, deg):
    step = deg2estep(deg)
    hstep = estep2ehex(step)
    return emsg(sio, addr, 'ma' + hstep)
