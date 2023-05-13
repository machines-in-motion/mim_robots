## mim_robots

This repository contains robot descriptions that are used in the machines in motion lab. It also contains APIs that can use the robot descriptions directly to create mujoco simulations. This code base will also contain dynamic_graph_head related API's in the future to enable 
easy sim2real robot transfer of code. 

## Dependencies
1. robot_descriptions - ```pip install robot_descriptions```
2. Mujoco - ```pip install mujoco```

## Code Infrastructure

### MiMRobotLoader 
This robot loader is interfaced with robot_descriptions. The robot loader uses the custom urdf and xml files for robots that are in the 
machines in motion labs. For other robots it utilizes robot_descriptions to download the necessary files. The MiMRobotLoader returns the pinocchio 
robot object (which contains model, data, collision_model ...) and the xml_path of the xml file of the robot. 

### Mujoco Env
The mujoc env class provides APIs to easily add robots, primitive objects (boxes), lights, cameras and use the mujoco simulator. This class is 
interfaced with the MiMRobotLoader. 

### Robot Envs 
These are specific case envs that are created based on previous project use cases. Feel free to add more envs so that they can be reused by others. 
Each robot env should only contain information regarding the robots to be loaded, and other objects that might be interacted with by the robot. 


## Possible Errors :
1. When trying to load a new robot if you see this error - ``` git.exc.InvalidGitRepositoryError ``` then run ``` rm -rf ~/.cache/robot_descriptions/jaxon_description``` and try again. This is an error that comes from robot_description. Link to issue - https://github.com/robot-descriptions/robot_descriptions.py

## Maintainer :

1. Avadesh Meduri
2. Huaijiang Zhu

## Copyrights

Copyright(c) 2023 New York University

## License

BSD 3-Clause License