# kinematics_model_generator

This repo parses a [URDF](http://wiki.ros.org/urdf) file using the API from https://github.com/ros/urdf. It then generates a kinematics model file. The grammar for this model is `Xtext`-based and is described in https://github.com/ipa320/kinematics-model.

## Installation

```
source /opt/ros/<ROS-DISTRO>/setup.bash
mkdir -p ~/kinematics_ws/src && cd ~/kinematics_ws/src
git clone git@github.com:ipa320/kinematics-model-parser.git
cd ~/kinematics_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source ~/kinematics_ws/devel/setup.bash
```

## Usage
The node accepts 2 positional arguments:  
`xacro-file`: absolute path to `XACRO` file to be parsed  
`model-file`: path to kinematics model file to be saved
```
rosrun urdf_model_generator xacro_model_generator xacro-file model-file
```
