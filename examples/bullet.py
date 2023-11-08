import pybullet as p

from mim_robots.robot_loader import load_bullet_wrapper
from mim_robots.pybullet.env import BulletEnvWithGround
from mim_robots.robot_list import MiM_Robots

env = BulletEnvWithGround(p.GUI)
# robot = load_bullet_wrapper("iiwa", controlled_joints =  ["A1", "A3", "A4", "A5",  "A7"])
robot = load_bullet_wrapper("teststand")

print(robot)
env.add_robot(robot)

# print(MiM_Robots.keys())

# for rname in MiM_Robots.keys():
#     if(rname != 'teststand'):
#         print("Adding "+rname)
#         robot = load_bullet_wrapper(rname)
#         env.add_robot(robot)

# # robot = load_bullet_wrapper("solo12")
# # print(robot)

for i in range(100000):
    env.step()