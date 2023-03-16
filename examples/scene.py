import mujoco
import glfw
import mujoco
from mujoco import _simulate
import numpy as np


model = mujoco.MjModel.from_xml_path("/home/georgezhang/devel/workspace/src/mim_robots/robots/kuka/kuka.xml")
data = mujoco.MjData(model)

Simulate = _simulate.Simulate

#   # The simulate object encapsulates the UI.

# simulate = Simulate()

#   # Initialize GLFW.
if not glfw.init():
    raise mujoco.FatalError('could not initialize GLFW')

# simulate.renderloop()
# atexit.register(glfw.terminate)

# if run_physics_thread:
# physics_thread = threading.Thread(
#     target=_physics_loop, args=(simulate, loader))
# physics_thread.start()

# simulate.renderloop()

# if run_physics_thread:
# physics_thread.join()