import pathlib
import matplotlib.pyplot as plt
import os
import mujoco
from mujoco_viewer import MujocoViewer
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin 
from dm_control import mjcf
import time

mj_model=mjcf.RootElement()

# chequered = mj_model.asset.add('texture', type='2d', builtin='checker', width=300,
#                     height=300, rgb1=[.2, .3, .4], rgb2=[.3, .4, .5])
# grid = mj_model.asset.add('material', name='grid', texture=chequered,
#                 texrepeat=[5, 5], reflectance=.2)
# mj_model.worldbody.add('geom', type='plane', size=[2, 2, .1], material = grid,  friction="0.3")
# mj_model.worldbody.add('light', pos=[10,0,10], dir=[1,1,])

xml_path="/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.xml"
# spawn_site = mj_model.worldbody.add('site', pos=np.array([5,0,0.1]))
robot = mjcf.from_path(xml_path)

# spawn_site.attach(robot_model)
mj_model.attach(robot)

model=mujoco.MjModel.from_xml_string(mj_model.to_xml_string)
data=mujoco.MjData(model)


viewer=MujocoViewer(model,data, width = 1200, height = 1000)

while data.time<10:
    mujoco.mj_step(model,data)
    viewer.render()


viewer.close