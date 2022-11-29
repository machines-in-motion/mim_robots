## This is an env that has a kuka robot with a gripper
## Author : Avadesh Meduri
## Date : 28/10/2022

from mujoco_env import MujocoWorld
from robot_loader import MiMRobotLoader

import pathlib
import numpy as np

class TriFingerEnv(MujocoWorld):

    def __init__(self):

        MujocoWorld.__init__(self)

        for x in [-3, 3]:
            self.add_light(pos=[x, -1, 2], dir=[-x, 1, -2])
        
        
        _, xml_path = MiMRobotLoader("trifinger0")     
        self.finger1 = self.add_body(xml_path, position = [0,0,0])
    
        _, xml_path = MiMRobotLoader("trifinger1")     
        self.finger2 = self.add_body(xml_path, position = [0,0,0])

        _, xml_path = MiMRobotLoader("trifinger2")     
        self.finger3 = self.add_body(xml_path, position = [0,0,0])

        ## The geom id contain index of the tip finger meshes and are used to compute contact forces
        self.geom_id = [3,6,9]

    def init_physics(self, initialize_renderer = False):
        """
        Creates physics. This is done outside the mujoco base class because it allows to set up the camera
        """
        self.create_physics(initialize_renderer = initialize_renderer)
        if self.initialize_renderer:
            self.set_camera_view(1, 90, 10, self.finger1)

    def get_trifinger_state(self):

        return self.get_states(self.finger1),  self.get_states(self.finger2), self.get_states(self.finger3)

    def reset_trifinger_state(self, q, v):
        self.reset_state([self.finger1, self.finger2, self.finger3] , q, v)

    def send_trifinger_torque(self, tau1, tau2, tau3):
        self.send_torque(self.finger1, tau1)
        self.send_torque(self.finger2, tau2)
        self.send_torque(self.finger3, tau3)
            
    def get_finger_contact(self, id):
        """
        Input:
            id : which finger (1,2 or 3)
        """
        return self.compute_contact_info(self.geom_id[id-1])
