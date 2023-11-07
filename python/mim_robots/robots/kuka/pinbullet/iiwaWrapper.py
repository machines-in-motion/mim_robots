"""iiwawrapper

Solo8 pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
import pybullet 
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_kuka.config import IiwaConfig

dt = 1e-3


class IiwaRobot(PinBulletWrapper):
    '''
    Pinocchio-PyBullet wrapper class for the KUKA LWR iiwa 
    '''
    def __init__(self, config, pos=None, orn=None): 

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.0]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(config.meshes_path)
        self.urdf_path = config.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=True)
        pybullet.getBasePositionAndOrientation(self.robotId)
        
        # Create the robot wrapper in pinocchio.
        self.pin_robot = config.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, 
                                    ji, 
                                    linearDamping=.04,
                                    angularDamping=0.04, 
                                    restitution=0.0, 
                                    lateralFriction=0.5)

        self.base_link_name = "iiwa_base"
        self.end_eff_ids = []
        controlled_joints = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
        self.end_eff_ids.append(self.pin_robot.model.getFrameId('contact'))
        self.nb_ee = len(self.end_eff_ids)
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.          
        super(IiwaRobot, self).__init__(
            self.robotId, 
            self.pin_robot,
            controlled_joints,
            ['EE'],
            useFixedBase=True)
        self.nb_dof = self.nv
        
    def forward_robot(self, q=None, dq=None):
        if q is None:
            q, dq = self.get_state()
        elif dq is None:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def start_recording(self, file_name):
        self.file_name = file_name
        pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)

    def stop_recording(self):
        pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, self.file_name)