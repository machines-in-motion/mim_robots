## Demo of the tri finger env
## Author : Avadesh Meduri
## Date : 29/11/2022
#%%
import pathlib
import matplotlib.pyplot as plt
import os
import mujoco 
import pinocchio as pin
import numpy as np
python_path = pathlib.Path('.').absolute().parent/'python'
os.sys.path.insert(1, str(python_path))

#from robot_envs.trifinger_env import TriFingerEnv

from robot_envs.kuka_env import KukaEnv



#sim = TriFingerEnv()

sim = KukaEnv(with_gripper=False)
sim.add_box("box", [0.05, 0.05, 0.1], [1,0,0,1], [0,-0.00,0.1], 0.5, [0.00083, 0.00083, 0.00083], 0.3)
model=sim.probot
data=model.data


# compute derivatives with respect to dynamics: 
Mmodel = mujoco.MjModel.from_xml_path("/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.xml")
Mdata= mujoco.MjData(Mmodel)
print(Mdata.qpos)

A=np.zeros((2*Mmodel.nv,2*Mmodel.nv))
B=np.zeros((2*Mmodel.nv, Mmodel.nu))
eps=1e-6
centered=True
mujoco.mjd_transitionFD(Mmodel, Mdata, eps, centered, A ,B, None,None) 
print("A is", A, "B is", B)



sim.init_physics(initialize_renderer = True)


# test controller 
P=100
D=1

q_res=np.zeros(7).T
dq_res=np.zeros(7).T
for i in range(10000):
    # simple PD controller 
    q, dq=sim.get_kuka_state()
    q_res=np.c_[q_res,q]
    dq_res=np.c_[dq_res,dq]
    
    q_des=np.array([np.pi,np.pi/4,0,0,0,0,0])
    #print(q_des,q)

    dq_des=np.zeros_like(dq)
    tau=P*(q_des-q)+D*(dq_des-dq)

    sim.send_kuka_torque(tau)

# inverse kinematics 


    sim.step()

# q_res=np.delete(q_res,0,1)
# dq_res=np.delete(dq_res,0,1)
# t=np.linspace(0,10,q_res.shape[1])
# print(q_res[:,0],dq_res[:,0])
# print(len(q_res),len(dq_res))
# #%%
# import matplotlib.pyplot as plt
# plt.plot(t,q_res[1,:])
# plt.show
# plt.plot(t,dq_res[1,:])
# plt.show
# # %%
