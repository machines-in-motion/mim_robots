## This class wraps around robot_description to allow use of MIM XML files 
## AKA - Tri fingers, kuka iiwa and Solo12. 
## Author : Avadesh Meduri
## Date : 29/11/2022

import pathlib
import pinocchio as pin

import os
from importlib import import_module  # type: ignore

def load_robot(xml_path, urdf_path, mesh_dir):
    if urdf_path:
        try:
            mesh_dir = str(pathlib.Path('.').absolute().parent.parent) + '/'
            pin_robot = pin.RobotWrapper.BuildFromURDF(
                                                filename=urdf_path,
                                                package_dirs=mesh_dir,
                                                root_joint=None,
                                                )
        except:
            print("Error: something wrong with urdf")
            pin_robot = None
    else:
        pin_robot = None
        
    return pin_robot, xml_path

def MiMRobotLoader(robot_name : str):
    """
    Input:
        robot_name : the name of the robot you want to load
    Output:
        pinocchio robot wrapper, mujoco xml_path
    """
    
    python_path = pathlib.Path('.').absolute().parent
    mesh_dir = str(pathlib.Path('.').absolute().parent.parent) + '/'
    
    if robot_name == "iiwa":
        xml_path = str(python_path) + '/robots/kuka/kuka.xml'   
        urdf_model_path = str(python_path) + "/robots/kuka/kuka.urdf"
        pin_robot, mjmodel = load_robot(xml_path, urdf_model_path, mesh_dir)

    elif robot_name == "iiwa_gripper":
        xml_path = str(python_path) + '/robots/kuka/kuka_gripper.xml'   
        urdf_model_path = str(python_path) + "/robots/kuka/kuka_gripper.urdf"
        pin_robot, mjmodel = load_robot(xml_path, urdf_model_path, mesh_dir)

    elif robot_name == "trifinger0":
        xml_path = str(python_path) + '/robots/trifinger/nyu_finger_triple0.xml'   
        urdf_model_path = str(python_path) + "/robots/trifinger/nyu_finger_triple0.urdf"
        pin_robot, mjmodel = load_robot(xml_path, urdf_model_path, mesh_dir)


    elif robot_name == "trifinger1":
        xml_path = str(python_path) + '/robots/trifinger/nyu_finger_triple1.xml'   
        urdf_model_path = str(python_path) + "/robots/trifinger/nyu_finger_triple1.urdf"
        pin_robot, mjmodel = load_robot(xml_path, urdf_model_path, mesh_dir)

    elif robot_name == "trifinger2":
        xml_path = str(python_path) + '/robots/trifinger/nyu_finger_triple2.xml'   
        urdf_model_path = str(python_path) + "/robots/trifinger/nyu_finger_triple2.urdf"
        pin_robot, mjmodel = load_robot(xml_path, urdf_model_path, mesh_dir)

    elif robot_name == "teststand":
        xml_path = str(python_path) + '/robots/teststand/teststand.xml'   
        urdf_model_path = str(python_path) + "/robots/teststand/teststand.urdf"
        pin_robot, mjmodel = load_robot(xml_path, urdf_model_path, mesh_dir)

    else:

        try:
            mj_name = robot_name + "_mj_description"
            mjmodule = import_module(f"robot_descriptions.{mj_name}")
            MJCF_PATH = mjmodule.MJCF_PATH
        except ModuleNotFoundError:
            print("Error: Mujoco Robot description not found ...")
            MJCF_PATH = None
        try:
            pin_name = robot_name + "_description"
            module = import_module(f"robot_descriptions.{pin_name}")
            package_dirs = [
                module.PACKAGE_PATH,
                module.REPOSITORY_PATH,
                os.path.dirname(module.PACKAGE_PATH),
                os.path.dirname(module.REPOSITORY_PATH),
                os.path.dirname(module.URDF_PATH), 
            ]
            URDF_PATH = module.URDF_PATH
        except ModuleNotFoundError:
            print("Error: Pinocchio Robot description not found ...")
            URDF_PATH = None
            package_dirs = None

        pin_robot, mjmodel = load_robot(MJCF_PATH, URDF_PATH, package_dirs)
        
    return pin_robot, mjmodel 

