# C025_FYP

This repository is the program for the Final Year Project. This ROS2 repository contains the files and programs used to create a dynamic pick and place system on a moving conveyor belt using a UR10 and ZED Stereo Camera.

### Prerequisites:
Ubuntu Version: 22.04

ROS2 Version : Humble Hawksbill

### Required repositories:
UR ROS2 Driver

```
sudo apt-get install ros-humble-ur
```

PyModBus

```
pip install pymodbus
```

## Quick Start
First, move to source folder of the workspace and clone the repository
```
git clone https://github.com/YanWenTan/C025_FYP.git
cd ..
colcon build
source install/setup.bash
```

## Launch File Functions
### Setup Initialisation
```
ros2 launch moveit_script setup_init.launch.py
```

This launch file will start the following and launch Rviz:
```
UR ROS2 Driver    
UR ROS2 Moveit Config    
ZED ROS2 Wrapper    
3-Finger Gripper Service    
Static Transform Publisher
```
### Object Detection
```
ros2 launch moveit_script obj_detection.launch.py
```

This launch file will first call the gripper service to initialise it.

Next, it will listen to topics from the ZED ROS2 Wrapper and start OpenCV. When an object is detected, it will publish the transformed pose and orientation to the `/transformed_pose` topic.

### Moveit Script
```
ros2 launch moveit_script moveit.launch.py
```
This launch file will create the MoveGroup Interface and listen to the `/transformed_pose` topic. When a pose is received, it will plan a trajectory to grasp the object.

## Additional Notes
### UR10 Joint Limits
If used with the default joint limit configurations, the path planning may occasionally plan a very large detour. This can be resolved by editing the `joint_limits.yaml` file of the UR10 in the file path `Comouter/opt/ros/humble/share/ur_description/config/ur10/joint_limits.yaml`.

The `max_position` and `min_position` for each joint are as follows:
```
shoulder_pan_joint
    max_position: !degrees  190
    min_position: !degrees -190
shoulder_lift_joint
    max_position: !degrees  10
    min_position: !degrees -159
elbow_joint
    max_position: !degrees  189
    min_position: !degrees -9
wrist_1_joint
    max_position: !degrees  9
    min_position: !degrees -189
wrist_2_joint
    max_position: !degrees  189
    min_position: !degrees -189
wrist_3_joint
    max_position: !degrees  189
    min_position: !degrees -189
```
### Static/Dynamic Modes
Both static and dynamic nodes can be found in the `moveit_script.cpp` file found in the file path `fyp_ws_v3/src/moveit_script/src/moveit_script.cpp`.

To enable dynamic grasping, lines 131 to 155 must be uncommented while lines 158 to 175 must be commented and vice versa for static grasping.

### GitHub Links for ROS2 packages used in project
Universal_Robots_ROS2_Driver from UniversalRobots:
```
https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
```
zed-ros2-wrapper from stereolabs:
```
https://github.com/stereolabs/zed-ros2-wrapper
```
robotiq_3f_gripper_ros2_stack from andreasHovaldt:
```
https://github.com/andreasHovaldt/robotiq_3f_gripper_ros2_stack
```
