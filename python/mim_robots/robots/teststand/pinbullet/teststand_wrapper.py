import pathlib
import os
python_path = pathlib.Path('.').absolute().parent/'python'
os.sys.path.insert(1, str(python_path))


import numpy as np
import time
import os
import pybullet
from mim_robots.pybullet.wrapper import PinBulletWrapper
import pinocchio as pin

dt = 1e-3

class TeststandRobot(PinBulletWrapper):

    def __init__(self, robotinfo, pos=None, orn=None, fixed_height=False):
        print(robotinfo)
        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.robotId = pybullet.loadURDF(
            robotinfo.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=True,
        )

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        self.pin_robot = pin.RobotWrapper.BuildFromURDF(
                                            filename=robotinfo.urdf_path,
                                            package_dirs=robotinfo.mesh_dir,
                                            root_joint=None,
                                            )

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.base_link_name = "base_link"
        controlled_joints = ["joint_z", "HFE", "KFE"]
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.
        super(TeststandRobot, self).__init__(
            self.robotId, self.pin_robot, controlled_joints, ["END"], useFixedBase=True
        )

        self.nb_dof = self.nv

        if fixed_height:
            pybullet.createConstraint(
                self.robotId,
                0,
                -1,
                -1,
                pybullet.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0.0],
                [0, 0, fixed_height],
            )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)