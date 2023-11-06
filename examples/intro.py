import pathlib
import os
python_path = pathlib.Path('.').absolute().parent/'python'
os.sys.path.insert(1, str(python_path))

from mim_robots.robot_loader import load_pinocchio_wrapper

load_pinocchio_wrapper("iiwa")
# load_pinocchio_wrapper("cassie")