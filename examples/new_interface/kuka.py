## Kuka demo
## Author : Avadesh Meduri & Huaijiang Zhu
## Date : 17/03/2022

import pathlib
import os

python_path = pathlib.Path('.').absolute().parent.parent/'python'
os.sys.path.insert(1, str(python_path))

from mujoco_env import MujocoWorld
from robot_loader import MiMRobotLoader
import mujoco
import time

xml_path="/home/ameduri/pydevel/mim_robots/robots/kuka/kuka.xml"
model=mujoco.MjModel.from_xml_path(xml_path)
sim = MujocoWorld(model, view_sim = True)

for i in range(1000):
    t1 = time.time()
    sim.step()
    t2 = time.time()
