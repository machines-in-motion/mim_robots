import pybullet as p

from mim_robots.robot_loader import load_bullet_wrapper
from mim_robots.pybullet.env import BulletEnvWithGround
from mim_robots.robot_list import MiM_Robots

env = BulletEnvWithGround(p.GUI)

# # TEST single robot load
# robot = load_bullet_wrapper("solo12")
# env.add_robot(robot)


# # TEST ALL ROBOTS
# print(MiM_Robots.keys())
# for rname in MiM_Robots.keys():
#     if(rname != 'teststand'):
#         print("Adding "+rname)
#         robot = load_bullet_wrapper(rname)
#         env.add_robot(robot)

# TEST robots with reduced model
robot = load_bullet_wrapper("iiwa", locked_joints = ["A2","A7"])
env.add_robot(robot)
# robot = load_bullet_wrapper("iiwa_ft_sensor_shell", locked_joints = ["A2","A7"])
# env.add_robot(robot)
# robot = load_bullet_wrapper("iiwa_ft_sensor_ball", locked_joints = ["A2","A7"])
# env.add_robot(robot)
# robot = load_bullet_wrapper("iiwa_gripper", locked_joints = ["A2","A7"])
# env.add_robot(robot)
robot = load_bullet_wrapper("solo12", locked_joints = ["FL_HAA"])
print(robot.pin_robot.model)
env.add_robot(robot)


for i in range(100000):
    env.step()