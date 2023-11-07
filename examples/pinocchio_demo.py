from mim_robots.robot_loader import load_pinocchio_wrapper, get_robot_list

print(get_robot_list())


robot = load_pinocchio_wrapper("solo12")
print(robot.model)
# load_pinocchio_wrapper("iiwa")
# load_pinocchio_wrapper("cassie")


