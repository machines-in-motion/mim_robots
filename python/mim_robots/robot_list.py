## This file contains all the robots in MiM
from dataclasses import dataclass
import importlib.resources

@dataclass
class RobotInfo:
    urdf_path : str
    mesh_dir : str
    xml_path : str
    fixed_base : bool

with importlib.resources.path(__package__, "robot_list.py") as p:
    package_path = p.parent.absolute()


resources_path = str(package_path) + '/robots/'
mesh_dir = resources_path

# # TODO: MOVE THESE TO A YAML FILE
# mim_robots = ["iiwa", "iiwa_ft_sensor_shell", "iiwa_ft_sensor_ball", "iiwa_gripper", "teststand", "trifinger0", "trifinger1", "trifinger2"]


MiM_Robots = {

    "iiwa" : RobotInfo(resources_path + "kuka/urdf/iiwa.urdf", \
                    resources_path + "kuka/meshes/kuka", \
                    None, True),


}



