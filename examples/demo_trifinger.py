## Demo of the tri finger env
## Author : Avadesh Meduri
## Date : 29/11/2022

import pathlib
import os

python_path = pathlib.Path('.').absolute().parent/'python'
os.sys.path.insert(1, str(python_path))

from robot_envs.trifinger_env import TriFingerEnv


sim = TriFingerEnv()
sim.add_box("box", [0.05, 0.05, 0.1], [1,0,0,1], [0,-0.00,0.1], 0.5, [0.00083, 0.00083, 0.00083], 0.3)

sim.init_physics(initialize_renderer = True)

for i in range(10000):
    sim.step()