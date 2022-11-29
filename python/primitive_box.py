## This class contains methods to add a box primitive in simulation
## obtain details about it such as surfaces, collision etc
## Author : Avadesh Meduri
## Date : 2/11/2022

import numpy as np
import itertools
# PyMJCF
from dm_control import mjcf
# Access to enums and MuJoCo library functions.
from dm_control.mujoco.wrapper.mjbindings import enums
from dm_control.mujoco.wrapper.mjbindings import mjlib

class PrimitiveBox:

    def __init__(self):

        self.indices = [[0,1,2,3], [4,5,6,7], [0,2,4,6], [1,3,5,7], [0,1,4,5], [2,3,6,7]]

    def add_box(self, name : str, size : list, rgba : list, pos : list, mass : float, \
                            intertia : list, friction : float):
        """
        This function adds a primitive shaped object like box, capsule, sphere into the simulation
        Input:
            size : size of the primitive
            rgba : color of the primitive
            pos : initial position 
            friction : friction of on surface
        """
        primitive = self.mj_model.worldbody.add('body', name=name, pos = pos)
        primitive.add('joint', type="free", name=name)
        primitive.add('geom', name=name, type="box", size=size, rgba=rgba, friction=str(friction))
        primitive.add("inertial", pos = [0,0,0], mass=mass, diaginertia=intertia)
        self.object_joint_indices[name] = [self.index[0], self.index[0] + 7]
        self.index[0] += 7 
        

    def get_box_location(self, name):

        return self.physics.named.data.xpos[name]

    def get_point_normal_box_surface(self, name, nb_s, interp : list):
        """
        This function return a point in the world frame that lies on chosen surface of the 
        cube and the corresponding normal to the surface at that point
        Input :
            name : name of the box
            nb_s : surface number (left surface, right most ...)
            interp : interpolated location of the point wrt the boundaries of the surface
        """
        assert np.sum(interp) == 1.0

        box_pos = self.physics.named.data.geom_xpos[name]
        box_mat = self.physics.named.data.geom_xmat[name].reshape(3, 3)
        box_size = self.physics.named.model.geom_size[name]
        offsets = np.array([-1, 1]) * box_size[:, None]
        xyz_local = np.stack(itertools.product(*offsets)).T
        xyz_global = (box_pos[:, None] + box_mat @ xyz_local)

        point = np.asarray(interp@xyz_global[:,self.indices[nb_s]].T).flatten()
        normal = np.asarray([0.25, 0.25, 0.25, 0.25]@xyz_global[:,self.indices[nb_s]].T).flatten() - self.get_box_location(name)
        normal /= np.linalg.norm(normal)

        return point.round(3), normal.round(3)



    