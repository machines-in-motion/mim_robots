import mujoco
from mujoco_viewer import MujocoViewer
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin
import time 

# inside this file: 1.create kuka loader function that creates mujoco setup
# 2. take derivatives in mujoco and time it using timing technique read crocoddyle and do simulation 



# functin that loads kuka 
def load_kuka():
    xml_path="/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.xml"
    urdf_path="/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.urdf"
    model=mujoco.MjModel.from_xml_path(xml_path)
    data=mujoco.MjData(model)
    pmodel=pin.buildModelFromUrdf(urdf_path)
    pdata=pmodel.createData()
    viewer=MujocoViewer(model,data,width=1200, height=1000)
    viewer.cam.distance=5.0

    return model,data,pmodel,pdata,viewer


model, data, pmodel, pdata, viewer=load_kuka()


# function that takes derivatives with mujoco num_diff
def derivative_dynamics(model,data,eps):
    A=np.zeros((2*model.nv, 2*model.nv))
    B=np.zeros((2*model.nv, model.nu))
    centered=True
    mujoco.mjd_transitionFD(model,data,eps,centered,A,B,None,None)
    return A,B


P=40
D=0.7
while data.time<10:
    st=time.time()
    #A, B=derivative_dynamics(model,data,1e-6)
    q, dq=data.qpos, data.qvel
    tau=np.ones(7)
    pin.computeABADerivatives(pmodel,pdata,q,dq,tau)


    
    et=time.time()
    
    #print(et-st)

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
    st1=time.time()
    tau=pin.rnea(pmodel,pdata,q,dq,a_des)
    et1=time.time()
    #print(et1 - st1)
    pin.forwardKinematics(pmodel,pdata,q,dq,np.zeros(model.nv))
    pin.updateFramePlacements(pmodel,pdata)
    print(pdata.oMf[pmodel.getFrameId("L7")].translation)

    data.ctrl=tau

    mujoco.mj_step(model,data)
    viewer.render()
    

    