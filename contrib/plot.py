#!/usr/bin/env python3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# Read data from file
data = np.genfromtxt(sys.argv[1], delimiter=",", names=["x", "y"])
ang = data['x']
pwr = data['y']

# Plot data
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set(xlabel="Angle (deg)",
       ylabel="Power (W)")
ax.plot(ang,pwr/np.max(pwr),'ko')
plt.savefig("plot.png", bbox_inches='tight',dpi=300)
