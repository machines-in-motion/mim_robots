## This class provides helper functions for mujoco with pinocchio
## Author : Avadesh Meduri
## Date : 21/10/2022

import numpy as np
import pinocchio as pin
import time

import mujoco

# PyMJCF
from dm_control import mjcf
# Access to enums and MuJoCo library functions.
from dm_control.mujoco.wrapper.mjbindings import enums
from dm_control.mujoco.wrapper.mjbindings import mjlib

import glfw

from dm_control import _render
from dm_control import mujoco
from dm_control.viewer import gui
from dm_control.viewer import renderer
from dm_control.viewer import viewer
from dm_control.viewer import views

from mujoco_render import MujocoRender
from primitive_box import PrimitiveBox

_MAX_FRONTBUFFER_SIZE = 2048
_MAX_FRONTBUFFER_SIZE = 2048


class MujocoWorld(MujocoRender, PrimitiveBox):
    
    def __init__(self, with_ground = True):
        """
        Input:
            xml_path : the path to the xml file of the robot
            with_ground : create the world with the ground
        """
        ## creating model        
        self.mj_model = mjcf.RootElement()
        self.object_joint_indices = {}
        self.object_actuator_indices = {}
        self.index = [0,0]
       
        chequered = self.mj_model.asset.add('texture', type='2d', builtin='checker', width=300,
                            height=300, rgb1=[.2, .3, .4], rgb2=[.3, .4, .5])
        grid = self.mj_model.asset.add('material', name='grid', texture=chequered,
                        texrepeat=[5, 5], reflectance=.2)
        self.mj_model.worldbody.add('geom', type='plane', size=[2, 2, .1], material = grid,  friction="0.3")

        PrimitiveBox.__init__(self)
        
    def add_light(self, pos : list, dir : list):
        """
        Adds a light in the scene 
        pos : position of the light
        dir : direction of the light
        """
        assert len(pos) == len(dir)
        self.mj_model.worldbody.add('light', pos=pos, dir=dir)

    def add_body(self, xml_path : str, position : list, isfreejoint = False):
        """
        Add a robot, object etc into the world
        Input:
            xml_path : xml_path of the object/robot
            position : the position where the object should be initialized in the world
            nb_joints : number of joints
            nb_actuators : number of actuated joints to control
            isfreejoint : is it a floating object that is free to move
        """
        spawn_site = self.mj_model.worldbody.add('site', pos=position)
        model = mjcf.from_path(xml_path)
        model_name = str(model.model)
        if isfreejoint:
            spawn_site.attach(model).add('freejoint')
        else:
            spawn_site.attach(model)

        mjmodel = mujoco.MjModel.from_xml_path(xml_path) #
        nb_joints, nb_actuators = mjmodel.nq, mjmodel.nv   
        self.object_joint_indices[model_name] = [self.index[0], self.index[0] + nb_joints]
        self.object_actuator_indices[model_name] = [self.index[1], self.index[1] + nb_actuators]
        self.index[0] += nb_joints 
        self.index[1] += nb_actuators

        return model_name

    def create_physics(self, initialize_renderer = False, dt = 0.001):
        """
        Creates the mujoco physics object (simulator)
        Note : Ensure that the mj_model is build completely before calling this method
        Input :
            dt : discretization of the simulation
        """
        self.physics = mjcf.Physics.from_mjcf_model(self.mj_model)
        self.physics.model.opt.timestep = dt
        self.dt = dt
        self.initialize_renderer = initialize_renderer
        if initialize_renderer:
            self.init_renderer()
    
    def init_renderer(self, frame_rate = 24, use_touchpad = True, 
                            width=768, height=576, title="Mujoco Simulator"):
        MujocoRender.__init__(self, frame_rate = 24, use_touchpad = True, 
                            width=768, height=576, title="Mujoco Simulator")

    def step(self, fastforward : int = 1):
        """
        This steps the simulation
        Input:
            fastforward : how many times faster than real time is desired when visualizing physics
        """
        self.physics.step()
        if self.initialize_renderer:
            self.viz_physics(fastforward=fastforward)

            
    def get_states(self, model_name : str):
        """
        Returns the joint position and velocity of the object of interest
        Input:
            model_name : name of the robot/object for which data is needed
        """
        indices = self.object_joint_indices[model_name]
        return self.physics.data.qpos[indices[0]:indices[1]], self.physics.data.qvel[indices[0]:indices[1]]

    def reset_state(self, model_names : list, q : list, v : list):
        """
        Resets the joint position and velocity of the object of interest
        Input:
            model_names : names of the robot/object for which data is needed
            q : joint position to be reset to
            v : joint velocity to be reset to
        """

        with self.physics.reset_context():
            for model_name in model_names:
                indices = self.object_joint_indices[model_name]
                self.physics.named.data.qpos[indices[0]:indices[1]] = q
                self.physics.named.data.qvel[indices[0]:indices[1]] = v


    def send_torque(self, model_name : str, torque : list):
        """
        sets the joint torques in the actuated joints of object of interest
        Input:
            model_name : name of the robot/object for which data is needed
        """
        indices = self.object_actuator_indices[model_name]
        self.physics.named.data.ctrl[indices[0]:indices[1]] = torque

    def compute_contact_info(self, geom_id):
        """
        This function returns the contact force, position, frame 
        Note : assumes only one contact
        Note : the contact forces are in the contact frame. To convert muliply frame and force torque
        Input:
            geom_id : the object id of interest
        """
        forcetorque = np.zeros(6)
        for j,c in enumerate(self.physics.data.contact):
            if c.geom1 == geom_id or c.geom2 == geom_id:
                mjlib.mj_contactForce(self.physics.model.ptr, self.physics.data.ptr, 
                                    j, forcetorque)
                if c.geom2 == geom_id:
                    forcetorque *= -1
                return forcetorque, c.pos, c.frame.reshape(3,3)

        return forcetorque, None, None
