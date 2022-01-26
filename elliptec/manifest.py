import tomli
import re

from .controller import Elliptec

class ManifestError(Exception):
    pass


class Manifest:
    """Declarative manifest of motorized elements and abstraction over control code.
    
    If you wish to incorporate another element driver xyz, subclass Manifest and
    add your own methods:
    - init_xyz()
    - validate_xyz()
    - move_xyz(name, position)
    - close()
    using the xyz_ or _xyz_ namespace for data and methods
    """
    def __init__(self, path):
        with open(path, "rb") as f:
            self.toml = tomli.load(f)
        self._validate()
        self._init_controllers()

    def _init_controllers(self):
        self.controllers = {}
        init_methods = [m for m in dir(self) if re.match("^init_", m) is not None]
        for m in init_methods:
            getattr(self, m)()

    def _validate(self):
        """Runs all methods matching "^validate_" to validate input manifest"""
        validate_methods = [m for m in dir(self) if re.match("^validate_", m) is not None]
        for m in validate_methods:
            getattr(self, m)()

    def move(self, movetable):
        for name in movetable:
            el = [e for e in self.toml.elements if e["name"] == name].pop()
            getattr(self, "move_" + el["type"])(name, movetable[name])

    def close(self):
        """Runs all methods matching "^close_" to close controller objects"""
        close_methods = [m for m in dir(self) if re.match("^close_", m) is not None]
        for m in close_methods:
            getattr(self, m)()

class ElliptecManifest(Manifest):
    """Methods to control Elliptec motors"""
    def init_elliptec(self):
        self._elliptec_controllers = {}
        cal = {}
        for element in self.toml.elements:
            if element["type"] in ["rot", "stage28", "stage60"]:
                cid = element["control"]["controller_id"]
                addr = element["control"]["address"]
                # Collect controllers and addresses for initialization
                if cid not in self._elliptec_controllers:
                    self._elliptec_controllers[cid] = []
                    self._elliptec_controllers[cid].append(addr)
                elif cid in self._elliptec_controllers:
                    self._elliptec_controllers[cid].append(addr)
                # Collect calibration info
                x = None
                for k in ["fast_axis", "slow_axis", "zero_axis"]:
                    if k in element["data"].keys():
                        x = element["data"][k]
                if x != None:
                    cal[addr] = x
        self.elliptec_controllers = {}
        cal = {}
        for ctl in self._elliptec_controllers:
            addrs = self._elliptec_controllers[ctl]
            self.elliptec_controllers[ctl] = Elliptec(ctl, addrs, cal=cal)
    
    def validate_elliptec(self):
        for element in self.toml.elements:
            # Simple verification that elliptec elements have valid control table
            if element["type"] in ["rot", "stage28", "stage60"]:
                if "controller_id" not in element["control"]:
                    raise ManifestError("no elliptec controller_id")
                if "address" not in element["control"]:
                    raise ManifestError("no elliptec controller address")

    def move_rot(self, name, pos):
        self.move_elliptec(name, pos)

    def move_stage28(self, name, pos):
        self.move_elliptec(name, pos)

    def move_stage60(self, name, pos):
        self.move_elliptec(name, pos)

    def move_elliptec(self, name, pos):
        el = [e for e in self.toml.elements if e["name"] == name].pop()
        ctrl = el["control"]["controller_id"]
        addr = el["control"]["address"]
        self.elliptec_controllers[ctrl].calmove(addr, pos)

class ManualManifest(Manifest):
    """Integration of manually-operated waveplates into manifest/control code.
    
    Blocks with a prompt to move the waveplate to the given angle.
    Supports offsets.
    """
    def init_manual(self):
        self.manual_cal = {}
        for element in self.toml.elements:
            if element["type"] is "manual":
                # Collect calibration info
                x = None
                for k in ["fast_axis", "slow_axis", "zero_axis"]:
                    if k in element["data"].keys():
                        x = element["data"][k]
                if x != None:
                    self.manual_cal[element["name"]] = x

    def validate_manual(self):
        pass

    def move_manual(self, name, pos):
        try:
            x0 = self.manual_cal[name]
        except KeyError:
            x0 = 0
        input(f"Move {name} to {pos + x0} and press Enter...")