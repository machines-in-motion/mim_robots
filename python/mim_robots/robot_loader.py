## This class wraps around robot_description to allow use of MIM XML files 
## AKA - Tri fingers, kuka iiwa and Solo12. 
## Author : Avadesh Meduri
## Date : 29/11/2022

import importlib
from importlib import import_module  # type: ignore
import importlib_resources
from . robot_list import MiM_Robots

from robot_descriptions.loaders.pinocchio import load_robot_description

def get_robot_list():
    """
        Returns the list of robots available
    """    
    print("The list of available robots are : \n")
    for robot_name in MiM_Robots.keys():
        print(robot_name + ": " + MiM_Robots[robot_name].description)


def load_mujoco_model(robot_name: str):
    """
        Returns mujoco model
        Input:
            robot_name : the name of the robot you want to load
    """ 
    try:
        import mujoco
        from robot_descriptions.loaders.mujoco import load_robot_description
    except Exception as e: print(e)
    
    try:
        RobotInfo = MiM_Robots[robot_name]
        try:
            return mujoco.MjModel.from_xml_path(RobotInfo.xml_path)
        except Exception as e: print(e)    
    except:
        try:
            mj_name = robot_name + "_mj_description"
            return load_robot_description(mj_name)
        except Exception as e: print("No ROBOT", e)


def load_bullet_wrapper(robot_name):
    try:
        import pybullet
        from robot_descriptions.loaders.pybullet import load_robot_description
    except Exception as e: print(e)

    # try:
    if(robot_name == 'iiwa'):
        from mim_robots.robots.kuka.pinbullet.iiwaWrapper import IiwaRobot
        return IiwaRobot(MiM_Robots["iiwa"])
    elif(robot_name == 'iiwa_ft_sensor_shell'):
        from mim_robots.robots.kuka.pinbullet.iiwaWrapper import IiwaRobot
        return IiwaRobot(MiM_Robots["iiwa_ft_sensor_shell"])
    elif(robot_name == 'iiwa_ft_sensor_ball'):
        from mim_robots.robots.kuka.pinbullet.iiwaWrapper import IiwaRobot
        return IiwaRobot(MiM_Robots["iiwa_ft_sensor_ball"])
    elif(robot_name == 'teststand'):
        from mim_robots.robots.teststand.pinbullet.teststand_wrapper import TeststandRobot
        robot = TeststandRobot(MiM_Robots["teststand"])
    elif(robot_name == 'solo12'):
        from mim_robots.robots.solo12.pinbullet.solo12wrapper import Solo12Robot
        robot = Solo12Robot()
        return robot
    else:
        assert False
    # except:
    #     try:
    #         name = robot_name + "_description"
    #         return load_robot_description(name)
    #     except:
    #         print("Robot description not support for pybullet")

def load_pinocchio_wrapper(robot_name: str):
    """
    Returns pinocchio wrapper model
    Input:
        robot_name : the name of the robot you want to load
    """
    try:
        import pinocchio as pin
    except Exception as e: print(e)

    try:
        RobotInfo = MiM_Robots[robot_name]
        try:
            if not RobotInfo.fixed_base:
                return pin.RobotWrapper.BuildFromURDF(
                                            filename=RobotInfo.urdf_path,
                                            package_dirs=RobotInfo.mesh_dir,
                                            root_joint=None,
                                            )
            else:
                return pin.RobotWrapper.BuildFromURDF(
                                            RobotInfo.urdf_path,
                                            RobotInfo.mesh_dir,
                                            pin.JointModelFreeFlyer()
                                            )
        except Exception as e: print("ERROR in URDF:", e)
    except:
        try:  
            pin_name = robot_name + "_description"
            print(pin_name)
            return load_robot_description(pin_name)
        except Exception as e: print("No ROBOT", e)


        
