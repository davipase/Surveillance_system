# SURVEILLANCE SYSTEM - UAV MANAGER

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger) ??poi da vedere

This project enables the simulation of a UAV with a PX4 Autopilot both in the gazebo simulated environment and using an Optitrack motion capture system. Necessary Ubuntu 20.04 or later.


## Requirements:
- [ROS2 foxy] (desktop-full recommended)
- [ROS noetic] (for the simulated mocap node only) (desktop-full recommended)
- [Java SDK 11]
- [Fast DDS], which include FastDDS, Fast-RTPS-gen
- a separated [PX4 Autopilot] package

# How to use
Regardless of the use case, two more packages are necessary dor the correct functioning of the system: the PX4_msgs and the PX4_ros_com package

Clone the repository:
```sh
mkdir -p ~/UAV_manager/src && cd UAV_manager/src
git clone https://github.com/davipase/Surveillance_system.git
```
Clone additional packages:
```sh
git clone git https://github.com/PX4/px4_ros_com.git ~/UAV_manager/src/px4_ros_com
git clone git https://github.com/PX4/px4_ros_com.git ~/UAV_manager/src/px4_ros_com
```
Build the project
```sh
cd ~/UAV_manager
colcon build
source install/setup.bash
```

## 1 - Gazebo simulation with user input:
All the following commands must be executed on a separate shell window.

### First tab: px4 simulation
```sh
cd ~/PX4-Autopilot
colcon build
source install/setup.bash
make px4_sitl_rtps gazebo
```
line 2 needs to be executed only the first time the PX4-Autopilot package is build or after it is modified

### Second tab: micrortps
```sh
cd ~/UAV_manager
colcon build
source install/setup.bash
micrortps_agent -t UDP
```

### Third tab: offboard node
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run controllo offboard
```

### Fourth tab: input node
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run controllo comando
```


   [ROS2 foxy]: <https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2>
   [ROS noetic]: <http://wiki.ros.org/noetic/Installation/Ubuntu>
   [Fast DDS]: <https://docs.px4.io/main/en/ros/ros2_comm.html#install-fast-dds>
   [Java SDK 11]: <https://www.oracle.com/java/technologies/downloads/#java11>
   [PX4 Autopilot]: <https://docs.px4.io/main/en/dev_setup/building_px4.html>
