import pathlib
import matplotlib.pyplot as plt
import os

python_path = pathlib.Path('.').absolute().parent/'python'
print(python_path)
os.sys.path.insert(1, str(python_path))

import mujoco
from mujoco_viewer import MujocoViewer
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin 
from dm_control import mjcf
import time



model = mujoco.MjModel.from_xml_path("/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.xml")
# worldbody=model.worldbody
data = mujoco.MjData(model)
pmodel=pin.buildModelFromUrdf("/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.urdf")
pdata=pmodel.createData()
# model.light.add("light", directional="true" ,diffuse=".5 .5 .5" ,specular=".5 .5 .5")
# #print(model.light_active)
print(model.light_active)
model.light_pos = [1.5, 1.5, 2.5]

print(model.light_pos)
viewer=MujocoViewer(model,data, width = 1200, height = 1000)
viewer.cam.distance = 5.0


P=30
D=0.2


q_res=np.zeros(7).T
dq_res=np.zeros(7).T


while data.time < 10:
    st = time.time()
    t=data.time 
    q, dq=data.qpos, data.qvel 
    viewer._joints = True

    # q_res=np.c_[q_res,q]
    # dq_res=np.c_[dq_res,dq]
    q_des=np.zeros(7)
    q_des[1]=np.pi/18*np.sin(2*np.pi*t)
    dq_des=np.zeros(7)
    dq_des[1]=np.pi/18*2*np.pi*np.cos(2*np.pi*t)
    a_des=np.zeros(7)
    a_des[1]=-np.pi/18*2*np.pi*2*np.pi*np.sin(2*np.pi*t)
    
    a_des=a_des+P*(q_des-q)+D*(dq_des-dq)
    tau=pin.rnea(pmodel,pdata,q,dq,a_des)

    data.ctrl=tau
    
    mujoco.mj_step(model,data)
    viewer.render()
    #print(data.ctrl)
    et = time.time()
    print(et - st)






