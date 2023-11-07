import pybullet as p

from mim_robots.robot_loader import load_bullet_wrapper
from mim_robots.pybullet.env import BulletEnvWithGround


env = BulletEnvWithGround(p.GUI)
robot = load_bullet_wrapper("iiwa_ft_sensor_ball")
print(robot)
env.add_robot(robot)

for i in range(100000):
    env.step()