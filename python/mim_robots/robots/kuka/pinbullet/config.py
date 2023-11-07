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

    # @classmethod
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
    def __init__(self, end_eff=None): 
        '''
        end_eff must be in ['ft_sensor_shell', 'ft_sensor_ball', None]
        '''
        self.robot_family = "kuka"   
        self.robot_name = "iiwa"

        self.paths = find_paths(self.robot_name, end_eff=end_eff) 
        self.meshes_path = self.paths["package"]
        self.yaml_path = self.paths["dgm_yaml"]
        self.urdf_path = self.paths["urdf"]
        
        # Pinocchio model.
        self.robot_model = se3.buildModelFromUrdf(self.urdf_path)

        self.mass = np.sum([i.mass for i in self.robot_model.inertias])

        self.base_name = self.robot_model.frames[2].name

        # The number of motors, here they are the same as there are only revolute
        # joints.
        self.nb_joints = self.robot_model.nv

        self.joint_names = [ "A1",
                        "A2",
                        "A3",
                        "A4",
                        "A5",
                        "A6",
                        "A7" ]

        self.end_effector_names = ["contact"]

        # Mapping between the ctrl vector in the device and the urdf indexes.
        self.urdf_to_dgm = tuple(range(self.robot_model.nv))

        self.map_joint_name_to_id = {}
        self.map_joint_limits = {}
        for i, (name, lb, ub) in enumerate(
            zip(
                self.robot_model.names[1:],
                self.robot_model.lowerPositionLimit,
                self.robot_model.upperPositionLimit,
            )
        ):
            self.map_joint_name_to_id[name] = i
            self.map_joint_limits[i] = [float(lb), float(ub)]

        # Define the initial state.
        self.initial_configuration = [0.]*self.robot_model.nq
        self.initial_velocity = [0.]*self.robot_model.nv

        self.q0 = zero(self.robot_model.nq)
        self.q0[:] = self.initial_configuration
        self.v0 = zero(self.robot_model.nv)
        self.a0 = zero(self.robot_model.nv)

        # In case there is an ft sensor with custom mount piece :
        # Get the name of the piece to which the CAD origin is attached
        # This can be used to compute the sensor frame placement w.r.t. parent joint
        if('shell' in self.urdf_path):
            self.cad_origin_name = 'assembled_ee'
        elif('ball' in self.urdf_path):
            self.cad_origin_name = 'kuka_to_sensor_mount'
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