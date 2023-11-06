"""config

Store the configuration of the Kuka family robots.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_kuka.utils import find_paths

class KukaAbstract(object):
    """ Abstract class for KUKA robots """

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path)
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class IiwaConfig(KukaAbstract):
    '''
    Config class for the KUKA LWR iiwa
    '''
    robot_family = "kuka"   
    robot_name = "iiwa"

    paths = find_paths(robot_name, end_eff='ft_sensor_shell') # ft_sensor_shell
    meshes_path = paths["package"]
    yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]
    
    # Pinocchio model.
    robot_model = se3.buildModelFromUrdf(urdf_path)

    mass = np.sum([i.mass for i in robot_model.inertias])

    base_name = robot_model.frames[2].name

    # The number of motors, here they are the same as there are only revolute
    # joints.
    nb_joints = robot_model.nv

    joint_names = [ "A1",
                    "A2",
                    "A3",
                    "A4",
                    "A5",
                    "A6",
                    "A7" ]

    end_effector_names = ["contact"]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(robot_model.nv))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = [0.]*robot_model.nq
    initial_velocity = [0.]*robot_model.nv

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

    # In case there is an ft sensor with custom mount piece :
    # Get the name of the piece to which the CAD origin is attached
    # This can be used to compute the sensor frame placement w.r.t. parent joint
    if('shell' in urdf_path):
        cad_origin_name = 'assembled_ee'
    elif('ball' in urdf_path):
        cad_origin_name = 'kuka_to_sensor_mount'
    else:
        pass

class IiwaReducedConfig(IiwaConfig):
    '''
    Config class for the iiwa reduced model
    '''
    # Override build_robot_wrapper to generate reduced model
    @classmethod
    def buildRobotWrapper(cls, controlled_joints, qref):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot_full = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path)

        controlled_joints_ids = []
        for joint_name in controlled_joints:
            controlled_joints_ids.append(robot_full.model.getJointId(joint_name))
        # Joint names to lock
        uncontrolled_joints = [] # 27 = 34 - 6 (controlled) - 1(universe)
        for joint_name in robot_full.model.names[1:]:
            if(joint_name not in controlled_joints):
                uncontrolled_joints.append(joint_name)
        locked_joints_ids = [robot_full.model.getJointId(joint_name) for joint_name in uncontrolled_joints]
        # qref = se3.neutral(robot_full.model)
        reduced_model, [visual_model, collision_model] = se3.buildReducedModel(robot_full.model, 
                                                                              [robot_full.visual_model, robot_full.collision_model], 
                                                                              locked_joints_ids, 
                                                                              qref)      
        robot = se3.robot_wrapper.RobotWrapper(reduced_model, collision_model, visual_model)  
        return robot