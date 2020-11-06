# multirotor_geometry_control
* This is a Gazebo simulation package for ros 18.04. The package is migrated from the [rotorS](https://github.com/ethz-asl/rotors_simulator).
* The geometry controller of the multirotor is implemented in this simulation.
* The parameters of the UAV can be found in /integral_concurrent_learning_UAV/rotors_simulator/rotors_description/urdf/firefly.xacro.
* Gains used for geometric control are defined in /integral_concurrent_learning_UAV/rotors_simulator/rotors_gazebo/resource/lee_controller_firefly.yaml.

## Requirements
* Ubuntu 18.04 ros-kinetic
* Gazebo 9 simulator

```
sudo apt-get install ros-kinetic-joy 
sudo apt-get install ros-kinetic-octomap-ros 
sudo apt-get install ros-kinetic-mavlink 
sudo apt-get install python-wstool
sudo apt-get install python-catkin-tools 
sudo apt-get install protobuf-compiler 
sudo apt-get install libgoogle-glog-dev 
sudo apt-get install ros-kinetic-control-toolbox
```

## Usage
```
roslaunch rotors_gazebo firefly_one.launch
roslaunch rotors_gazebo controller_geometry.launch
```

