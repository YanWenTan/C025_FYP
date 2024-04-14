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

    #UR Driver
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ur_robot_driver'),
                                                   'launch/ur_control.launch.py')),
        launch_arguments= {'ur_type' : 'ur10',
                           'robot_ip' : '192.168.1.102',
                           'launch_rviz' : 'false'}.items()
    )

    #Create Gripper Service
    gripper_service_launch = Node(
        package= "robotiq_3f_gripper_ros2_control",
        executable= 'gripper_control_service_server'
    )

    #ZED Camera
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('zed_wrapper'),
                                                   'launch/zed_camera.launch.py')),
        launch_arguments= {'camera_model' : 'zed'}.items()
    )
    
    #Frame Transform
    frame_transform_launch = Node(
        package= "tf2_ros",
        executable= 'static_transform_publisher',
        arguments= ["0.64",     #x
                    "0.4",      #y
                    "1",        #z
                    "-0.271",   #qx
                    "0.653",    #qy
                    "0.271",    #qz
                    "0.653",    #qw
                    "world",    #frame_id
                    "map"]      #child_frame_id
    )

    #UR Moveit
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ur_moveit_config'),
                                                   'launch/ur_moveit.launch.py')),
        launch_arguments= {'ur_type' : 'ur10',
                           'launch_rviz' : 'true'}.items()
    )

    ld.add_action(ur_driver_launch)
    ld.add_action(gripper_service_launch)
    ld.add_action(zed_camera_launch)
    ld.add_action(frame_transform_launch)
    ld.add_action(ur_moveit_launch)

    return ld
