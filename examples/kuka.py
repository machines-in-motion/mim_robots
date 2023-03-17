## Kuka demo
## Author : Avadesh Meduri & Huaijiang Zhu
## Date : 17/03/2022

import pathlib
import os

python_path = pathlib.Path('.').absolute().parent/'python'
os.sys.path.insert(1, str(python_path))

from robot_loader import MiMRobotLoader
from mujoco_renderer import MujocoRenderer

import mujoco
import time


pin_robot, model = MiMRobotLoader("cassie")
# pin_robot, model = MiMRobotLoader("iiwa")

data = mujoco.MjData(model)
viewer=MujocoRenderer(model,data,width=1200, height=1000)

for i in range(1000):
    t1 = time.time()
    mujoco.mj_step(model,data)
    viewer.render()
    t2 = time.time()
