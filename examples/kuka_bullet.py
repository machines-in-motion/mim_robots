from mim_robot.robots.kuka.pinbullet.config import Iiwaconfig

config = Iiwaconfig()

robot = config.buildRobotWrapper()