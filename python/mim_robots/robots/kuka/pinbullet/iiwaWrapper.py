"""iiwawrapper
Iiwa pybullet interface using pinocchio's convention.
License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""
import pybullet 
from mim_robots.pybullet.wrapper import PinBulletWrapper
import pinocchio as pin
import numpy as np

dt = 1e-3

class IiwaRobot(PinBulletWrapper):
    '''
    Pinocchio-PyBullet wrapper class for the KUKA LWR iiwa 
    '''
    def __init__(self, robotinfo, controlled_joints, qref=np.zeros(7), pos=None, orn=None): 

        if controlled_joints == None:
            controlled_joints = ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]
        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.0]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.urdf_path = robotinfo.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=robotinfo.fixed_base)
        pybullet.getBasePositionAndOrientation(self.robotId)
        
        # Create the robot wrapper in pinocchio.
        robot_full = pin.RobotWrapper.BuildFromURDF(
                                            filename=robotinfo.urdf_path,
                                            package_dirs=robotinfo.mesh_dir,
                                            root_joint=None,
                                            )

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(self.robotId, 
                                    ji, 
                                    linearDamping=.04,
                                    angularDamping=0.04, 
                                    restitution=0.0, 
                                    lateralFriction=0.5)
            
        # Optionally reduce the model
        if(len(controlled_joints) < 7):
            # Optionally reduce the model (freeze joints)
            controlled_joints_ids = []
            for joint_name in controlled_joints:
                controlled_joints_ids.append(robot_full.model.getJointId(joint_name))

            # Joint names & pin ids to lock
            uncontrolled_joints = [] 
            for joint_name in robot_full.model.names[1:]:
                if(joint_name not in controlled_joints):
                    uncontrolled_joints.append(joint_name)
            locked_joints_ids = [robot_full.model.getJointId(joint_name) for joint_name in uncontrolled_joints]
            # print('Locked joints ids in pinocchio '+'('+str(len(locked_joints_ids))+') : ')
            
            # Build reduced model with ref posture for locked joints
            # Retain locked joints reference position for later
            qref_locked_map = {}
            for joint_name in uncontrolled_joints:
                idx = robot_full.model.getJointId(joint_name)-1
                qref_locked_map[joint_name] = qref[idx]
            # Make reduced model and wrapper
            reduced_model, [visual_model, collision_model] = pin.buildReducedModel(robot_full.model, 
                                                                                [robot_full.visual_model, robot_full.collision_model], 
                                                                                locked_joints_ids, 
                                                                                qref)   
            self.pin_robot = pin.robot_wrapper.RobotWrapper(reduced_model, collision_model, visual_model)  
            print("[pinbullet wrapper] REDUCED MODEL : ", self.pin_robot.model)

            # Get bullet map joint_name<->bullet_index
            bullet_joint_map = {}
            for ji in range(pybullet.getNumJoints(self.robotId)):
                bullet_joint_map[pybullet.getJointInfo(self.robotId, ji)[1].decode("UTF-8")] = ji
            # Get bullet ids of locked joints + subconfig
            if('root_joint' in uncontrolled_joints):
                uncontrolled_joints.remove('root_joint') # base treated in sim
            locked_joint_ids_bullet = np.array([bullet_joint_map[name] for name in uncontrolled_joints])
            qref_locked = [qref_locked_map[joint_name] for joint_name in uncontrolled_joints]
            # Lock the uncontrolled joints in position control in PyBullet multibody (full robot)
            for joint_name in uncontrolled_joints:
                # print("joint name : " + joint_name + " , bullet joint id = ", bullet_joint_map[joint_name])
                pybullet.resetJointState(self.robotId, bullet_joint_map[joint_name], qref_locked_map[joint_name], 0.)
            pybullet.setJointMotorControlArray(self.robotId, 
                                            jointIndices = locked_joint_ids_bullet, 
                                            controlMode = pybullet.POSITION_CONTROL,
                                            targetPositions = qref_locked,
                                            targetVelocities = np.zeros(len(locked_joint_ids_bullet)))
        # Otherwise full robot
        else:
            self.pin_robot = robot_full

        self.base_link_name = "iiwa_base"
        self.end_eff_ids = []
        self.end_eff_ids.append(self.pin_robot.model.getFrameId('contact'))
        self.nb_ee = len(self.end_eff_ids)
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.          
        super(IiwaRobot, self).__init__(
            self.robotId, 
            self.pin_robot,
            controlled_joints,
            ['EE'],
            useFixedBase=robotinfo.fixed_base)
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