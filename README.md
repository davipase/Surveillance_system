# SURVEILLANCE SYSTEM - UAV MANAGER

This project enables the simulation of a UAV with a PX4 Autopilot both in the gazebo simulated environment and using an Optitrack motion capture system. Necessary Ubuntu 20.04 or later.


## Requirements:
- [ROS2 foxy] (desktop-full recommended)
- [ROS noetic] (for the simulated mocap node only) (desktop-full recommended)
- [Java SDK 11]
- [Fast DDS], which include FastDDS, Fast-RTPS-gen
- a separated [PX4 Autopilot] package, to which it is necessary to change the COM_RCL_EXCEPT parameter to 4

# How to use
Regardless of the use case, two more packages are necessary dor the correct functioning of the system: the PX4_msgs, PX4_ros_com and cpp-spline package

**Step 1: Clone the repository:**
```sh
mkdir -p ~/UAV_manager/src && cd UAV_manager/src
git clone https://github.com/davipase/Surveillance_system.git
```
**Step 2: Clone additional packages:**
```sh
git clone git https://github.com/PX4/px4_msgs.git ~/UAV_manager/src/px4_msgs
git clone git https://github.com/PX4/px4_ros_com.git ~/UAV_manager/src/px4_ros_com
git clone https://github.com/chen0040/cpp-spline.git ~/UAV_manager/src/cpp-spline
```
**Step 3: Move the message "Comando.msg" into the msg folder
```sh
cd ~/UAV_manager/src
mv Comando.msg px4_msgs/msg/Comando.msg
```
**Step 4: Build the project**
```sh
cd ~/UAV_manager
colcon build
source install/setup.bash
```

**Now the project is ready to be used.**
All the following commands must be executed on a separate shell window.

### Tab 1: px4 simulation
```sh
cd ~/PX4-Autopilot
colcon build
source install/setup.bash
make px4_sitl_rtps gazebo
```
> Note: line 2 needs to be executed only the first time the PX4-Autopilot package is build or after it is modified

### Tab 2: micrortps
```sh
cd ~/UAV_manager
colcon build
source install/setup.bash
micrortps_agent -t UDP
```

### Tab 3: offboard node
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run controllo offboard
```

### Tab 4: input node
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run controllo comando
```

## Simulated mocap node
With a few adjustments, it is possible to simulate a Motion Captur system inside the gaebo simulation

In the PX4_Autopoiolot package:
- Change the `SYS_MC_EST_GROUP` parameter to 2
- Change the `EKF2_AID_MASK` parameter to 24
- Change the `EKF2_HGT_MODE` to 3

This will change the source for the position data to the mocap node.
Then, in addition to tab 1..4, two more windows are needed:

### Tab 5: Mocap data
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run mocap mocap
```

### Tab 6: Mocap publisher
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run controllo mocap
```

## Optitrack mocap system
It is possible to replace the simulated Mocap node with a real optitrack system. To do so, it is necessary for the optitrack system and the computer to be connected to the same Wi-Fi connection and to install on your machine both ROS2 Foxy and ROS noetic

**Step 1: Download the ROS1 bridge**
```sh
cd ~/UAV_manager/src
git clone https://github.com/ros2/ros1_bridge.git ~/UAV_manager/src/ros1_bridge
```

**Step 2: download, build and start the [Natnet] package following the README instructions**

**Step 3: build the workspace**
> Note: in the following lines of code it is assumed that you installed ROS noetic and ROS2 Foxy to the default locations `/opt/ros/noetic` and `/opt/ros/foxy`. Otherwise, change them with the correct install path.

- build the ROS2 packages
```sh
cd ~/UAV_manager
colcon build --symlink-install --packages-skip ros1_bridge
surce /opt/ros/noetic/setup.bash
```
- build the ROS1 bridge
```sh
surce /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-skip ros1_bridge
source install/setup.bash
```
**Step 4: start the nodes**
- Start tabs 1..4 (don't execute `colcon build` while starting tab 2)
- Start the Optitrack Mocap node
in a different Tab, execute the following:
```sh
cd ~/UAV_manager
source install/setup.bash
ros2 run optitrack_mocap optitrack_mocap
```
- Run the [Natnet] node as shown in the README
- Run the ROS1 bridge

> Note: the *ros1_bridge* package needs an active roscore master to run. In this case, the roscore is already running from the *roslaunch* command used to start the Natnet node. If you want to run the bridge alone, remenber to run `roscore` in a different tab

- in one more tab, run the optitrack_mocap node with the command
```sh
ros2 run optitrack_mocap optitrack_mocap
```



   [ROS2 foxy]: <https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2>
   [ROS noetic]: <http://wiki.ros.org/noetic/Installation/Ubuntu>
   [Fast DDS]: <https://docs.px4.io/main/en/ros/ros2_comm.html#install-fast-dds>
   [Java SDK 11]: <https://www.oracle.com/java/technologies/downloads/#java11>
   [PX4 Autopilot]: <https://docs.px4.io/main/en/dev_setup/building_px4.html>
