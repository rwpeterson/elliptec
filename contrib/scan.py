#!/usr/bin/env python3

from os.path import abspath, dirname
import pyvisa
import random
import sys
from ThorlabsPM100 import ThorlabsPM100
from time import sleep

elliptec_dir = dirname(dirname(abspath(__file__)))
sys.path.append(elliptec_dir)
from ThorlabsELLx import *

def mprint(x): print(x, sep='', end='')    

# Connect to power meter
rm = pyvisa.ResourceManager()
inst = rm.open_resource('USB0::0x1313::0x8078::P0015227::0::INSTR', timeout=None)  # PM100 console
#inst = rm.open_resource('USB0::4883::32882::1902463::0::INSTR', timeout=None)     # PM100 no-console
pm = ThorlabsPM100(inst=inst)

pm.system.beeper.immediate()
pm.sense.power.dc.range.auto = "ON"
pm.input.pdiode.filter.lpass.state = 0
pm.sense.average.count = 1000
pm.sense.correction.wavelength = 1550

t = ThorlabsELLx("COM3", '0')

angles = list(range(0, 360, 1))
random.seed(42)
random.shuffle(angles)

data = {}
for angle in angles:
    t.absm('0', angle)
    sleep(0.5)
    data[angle] = pm.read
    mprint(str(angle) + ',' + str(data[angle]) + '\n')


