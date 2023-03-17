
import numpy as np
import mujoco

from mujoco_renderer import MujocoRenderer

class MujocoWorld(MujocoRenderer):

    def __init__(self, mj_model, view_sim = True):
        """
        mj_model : mujoco model
        view_sim : start the mujoco viewer to visualize simulation
        """

        self.rmodel = mj_model
        self.rdata = mujoco.MjData(self.rmodel)
        self.view_sim = view_sim
        if self.view_sim:
            self.viewer=MujocoRenderer(self.rmodel,self.rdata,width=1200, height=1000)

    def step(self):
        """
        This function steps the simulation
        """
        mujoco.mj_step(self.rmodel,self.rdata)
        if self.view_sim:
            self.viewer.render()