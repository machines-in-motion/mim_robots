## Demo for loading robots and using mujoco env
## Author : Avadesh Meduri
## Date : 29/11/2022

import pathlib
import os

python_path = pathlib.Path('.').absolute().parent/'python'
os.sys.path.insert(1, str(python_path))

from mujoco_env import MujocoWorld
from robot_loader import MiMRobotLoader

# pin_robot, mjpath = MiMRobotLoader("panda")
pin_robot, mjpath = MiMRobotLoader("iiwa")

tmp = MujocoWorld()
a1 = tmp.add_body(mjpath, [0,0,0], False)
tmp.create_physics(True)

for i in range(10000):
    tmp.step()