## Demo for loading robots and using mujoco env
## Author : Avadesh Meduri
## Date : 29/11/2022

import pathlib
import os

from mim_robots.mujoco_env import MujocoWorld
from mim_robots.robot_loader import MiMRobotLoader

pin_robot, mjpath = MiMRobotLoader("a1")
# pin_robot, mjpath = MiMRobotLoader("iiwa")

tmp = MujocoWorld(False)
a1 = tmp.add_body(mjpath, [0,0,0], True)
tmp.create_physics(True)

for i in range(10000):
    tmp.step()