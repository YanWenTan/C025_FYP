import os
import rclpy

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess

def generate_launch_description():

    ld = LaunchDescription()

    #Initialise Gripper
    gripper_init = ExecuteProcess(cmd=[[
        FindExecutable(name='ros2'),
        " service call ",
        "/Robotiq3FGripper/OutputRegistersService ",
        "robotiq_3f_gripper_ros2_interfaces/srv/Robotiq3FGripperOutputService ",
        '"{output_registers: {r_act: 1, r_mod: 1, r_gto: 1, r_atr: 0, r_pra: 0, r_spa: 255, r_fra: 0}}"',
    ]],
    shell=True
    )

    #Object Detection
    obj_detection = Node(
        package= "cv_ros2_test",
        executable= 'obj_detection_publisher'
    )

    #Pose Transform
    pose_transform = Node(
        package= "cv_ros2_test",
        executable= 'transform_pose_publisher'
    )

    ld.add_action(gripper_init)
    ld.add_action(obj_detection)
    ld.add_action(pose_transform)

    return ld
