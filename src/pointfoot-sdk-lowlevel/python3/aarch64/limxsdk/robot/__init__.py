import os
import platform

if platform.system() == "Linux":
    import ctypes
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_path = os.path.join(current_dir, "libpython3.8.so.1.0")
    ctypes.CDLL(lib_path)

from .RobotType import *
from ._robot import *
from .Rate import *
from .Robot import *
from .Joystick import *