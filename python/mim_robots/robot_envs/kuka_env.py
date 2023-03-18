## This is an env that has a kuka robot with a gripper
## Author : Avadesh Meduri
## Date : 24/10/2022

from envs.mujoco_env import MujocoWorld
from envs.expert_demonstration import ExpertDemonstration
import pathlib
import numpy as np


class KukaEnv(ExpertDemonstration, MujocoWorld):

    def __init__(self, with_gripper = True):

        MujocoWorld.__init__(self)
        for x in [-2, 2]:
            self.add_light(pos=[x, -1, 3], dir=[-x, 1, -2])

        python_path = pathlib.Path('.').absolute().parent.parent
        if with_gripper:
            xml_path = str(python_path) + '/robots/kuka/kuka_gripper.xml'        
            self.nb_joints = 13
            self.nb_actuators = 13
        else:
            xml_path = str(python_path) + '/robots/kuka/kuka.xml'        
            self.nb_joints = 7
            self.nb_actuators = 7

        self.kuka = self.add_body(xml_path, position = [0,0,0], nb_joints = self.nb_joints, nb_actuators= self.nb_actuators)

    def init_physics(self, initialize_renderer = False):
        self.create_physics(initialize_renderer=initialize_renderer)

    def get_kuka_state(self):

        return self.get_states(self.kuka)
    
    def reset_kuka_state(self, q, v):
        self.reset_state([self.kuka], q, v)

    def send_kuka_torque(self, tau):
        self.send_torque(self.kuka, tau)

    def step_mpc(self, pin_robot):
        """
        This function steps the simulation and calls the IOC weight computation at the correct times
        """
        q, dq = self.get_kuka_state()
        q_int, dq_int, a_int = pin_robot.compute_ctrl(q, dq)
        hor = len(q_int)
        for i in range(hor):
            tau = np.zeros(self.nb_joints)
            tau[:self.nb_joints] = pin_robot.compute_id_command(q[:self.nb_joints], dq[:self.nb_joints], q_int[i], dq_int[i], a_int[i])
            self.send_kuka_torque(tau)
            self.step()
            q, dq = self.get_kuka_state()
            
        