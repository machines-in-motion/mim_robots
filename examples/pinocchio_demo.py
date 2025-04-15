from mim_robots.robot_loader import load_pinocchio_wrapper, get_robot_list

# print(get_robot_list())


robot = load_pinocchio_wrapper("solo12")
print(robot.model)

robot = load_pinocchio_wrapper("iiwa", locked_joints = ["A2","A7"])
print(robot.model)

robot = load_pinocchio_wrapper("panda", locked_joints = ["panda_finger_joint1", "panda_finger_joint2"])
print(robot.model)