## mim_robots

This repository contains robot descriptions that are used in the Machines in Motion lab. It also contains APIs that can use the robot descriptions directly to create Mujoco or PyBullet simulations. This code base also contains dynamic_graph_manager configuration YAML files to enable easy transfer to real hardware.

## Dependencies
1. robot_descriptions - ```pip install robot_descriptions```
2. [pinocchio](https://github.com/stack-of-tasks/pinocchio) 
3. Mujoco - ```pip install mujoco```
4. PyBullet - ```pip install pybullet```

## Code Infrastructure
The 3 files in the example directory show how to use the package to load models for pinocchio, mujoco and pybullet. 

### MiMRobotLoader 
This robot loader is interfaced with robot_descriptions. The robot loader uses the custom urdf and xml files for robots that are in the 
machines in motion labs. For other robots it utilizes robot_descriptions to download the necessary files. The MiMRobotLoader returns the pinocchio robot object (which contains model, data, collision_model ...) and the xml_path of the xml file of the robot. 

### Robot Envs 
These are specific case envs that are created based on previous project use cases. Feel free to add more envs so that they can be reused by others. 
Each robot env should only contain information regarding the robots to be loaded, and other objects that might be interacted with by the robot. 


## Possible Errors :
1. When trying to load a new robot if you see this error - ``` git.exc.InvalidGitRepositoryError ``` then run ``` rm -rf ~/.cache/robot_descriptions/jaxon_description``` and try again. This is an error that comes from robot_description. Link to issue - https://github.com/robot-descriptions/robot_descriptions.py

## Maintainer :

1. Avadesh Meduri
2. Huaijiang Zhu
3. Sébastien Kleff

## Copyrights

Copyright(c) 2024 New York University

## License

BSD 3-Clause License
