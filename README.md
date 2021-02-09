# elliptec

Python interface to Thorlabs Elliptec motorized mounts

## Purpose

This project allows for easy control of Thorlabs Elliptec motorized
mounts in Python, using their documented serial commands. It also
affords some conveniences, such as:
- Detect the module type, scaling factor, etc., during initialization
- Perform frequency calibration and homing of the motors automatically
- Monitor the replies from modules, retry commands if necessary
  and correct for known communication bugs
- Keep track of per-module offsets, e.g. to rotate relative to a
  calibrated angle for a waveplate

## Getting started

```python
from elliptec import Elliptec

# Open controller at this serial address, communicate with modules at
# the hexadecimal addresses 0, 1, and 2 (for A-F use capital letters)
e = Elliptec('/dev/ttyUSB0', ['0', '1', '2'])

# Let's assume we have three waveplates, each with a fast axis at a known
# offset angle from the motor's zero position of 13, 42, and 7 degrees,
# respectively. Relative to these positions, we want to move to (0, 90, 22.5)

# We can move one-by-one:
e.moveabsolute('0', 13 + 0)
e.moveabsolute('1', 42 + 90)
e.moveabsolute('2', 7 + 22.5)

# Or, we can use the convenience setcal() and calmove() methods:
e.setcal(['0', '1', '2'], [13, 42, 7])
e.calmove(['0', '1', '2'], [0, 90, 22.5])

# Since we're using all the modules initialized above, we can omit addresses:
e.setcal([13, 42, 7])
e.calmove([0, 90, 22.5])

# Now it's easy to go to another position without carrying the offsets:
e.calmove([0, 45, -22.5])

# We're done here!
e.close()
```

## Installing / Developing

This project is currently not packaged on PyPI. The easiest way
to install is to clone the repo and install the local version in
editable mode with `pip`:

    git clone https://git.sr.ht/~rwp/elliptec
    cd elliptec
    pip3 install -e .

Now, any changes you make locally will automatically be applied in your
scripts. To update to a newer version, use `git` to pull changes as
usual into your local repo.

## Scope

Contributors currently have only used this to control the ELL14
rotation mount and ELL17 linear stage. Not all commands are currently
implemented.

## Contributions

Patches and comments are welcome, especially if you have another model
not listed above.

Please check that contributions are `flake8`-clean. From the project
directory, simply type `make lint` to check the appropriate files.
