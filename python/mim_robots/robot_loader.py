## This class wraps around robot_description to allow use of MIM XML files 
## AKA - Tri fingers, kuka iiwa and Solo12. 
## Author : Avadesh Meduri
## Date : 29/11/2022

import pathlib
import pinocchio as pin
import mujoco

import os
import importlib
from importlib import import_module  # type: ignore
import importlib_resources


# TODO: MOVE THESE TO A YAML FILE
mim_robots = ["iiwa", "iiwa_gripper", "teststand", "trifinger0", "trifinger1", "trifinger2"]

def find_mim_paths(robot_name: str):
    try:
        with importlib.resources.path(__package__, "robot_loader.py") as p:
            package_path = p.parent.absolute()
    except: # compatibility for Python < 3.7
        with importlib_resources.path(__package__, "robot_loader.py") as p:
            package_path = p.parent.absolute()
    resources_path = str(package_path) + '/robots/'
    mesh_dir = resources_path

    if robot_name == "iiwa":
        xml_path = str(resources_path) + 'kuka/kuka.xml'   
        urdf_model_path = str(resources_path) + "kuka/kuka.urdf"

    elif robot_name == "iiwa_gripper":
        xml_path = str(resources_path) + 'kuka/kuka_gripper.xml'   
        urdf_model_path = str(resources_path) + "kuka/kuka_gripper.urdf"

    elif robot_name == "trifinger0":
        xml_path = str(resources_path) + 'trifinger/nyu_finger_triple0.xml'   
        urdf_model_path = str(resources_path) + "trifinger/nyu_finger_triple0.urdf"

    elif robot_name == "trifinger1":
        xml_path = str(resources_path) + 'trifinger/nyu_finger_triple1.xml'   
        urdf_model_path = str(resources_path) + "trifinger/nyu_finger_triple1.urdf"

    elif robot_name == "trifinger2":
        xml_path = str(resources_path) + 'trifinger/nyu_finger_triple2.xml'   
        urdf_model_path = str(resources_path) + "trifinger/nyu_finger_triple2.urdf"

    elif robot_name == "teststand":
        xml_path = str(resources_path) + 'teststand/teststand.xml'   
        urdf_model_path = str(resources_path) + "teststand/teststand.urdf"

    return xml_path, urdf_model_path, mesh_dir

def load_robot(xml_path, urdf_path, mesh_dir):
    
    pin_robot = None
    mjmodel = None

    if urdf_path:
        try:
            pin_robot = pin.RobotWrapper.BuildFromURDF(
                                                filename=urdf_path,
                                                package_dirs=mesh_dir,
                                                root_joint=None,
                                                )
        except:
            print("Error: something wrong with urdf")
            pin_robot = None
        
    if xml_path:
        mjmodel = mujoco.MjModel.from_xml_path(xml_path)

    return pin_robot, mjmodel

def MiMRobotLoader(robot_name: str):
    """
    Input:
        robot_name : the name of the robot you want to load
    Output:
        pinocchio robot wrapper, mujoco xml_path
    """
    
  
    if robot_name in mim_robots:
        xml_path, urdf_model_path, mesh_dir = find_mim_paths(robot_name)
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

