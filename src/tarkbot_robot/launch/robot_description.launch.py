import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

def generate_launch_description():

    tarkbot_r20_akm = GroupAction([
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='foot_to_base',
            arguments=['0 ', '0', '0.074','0', '0','0','base_footprint','base_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.098 ', '0', '0.035','0', '0','0','base_link','laser_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.077', '0', '0.0805','0', '0','0','base_link','camera_link'],),
    ])
    
    tarkbot_r20_fwd = GroupAction([
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='foot_to_base',
            arguments=['0 ', '0', '0.125','0', '0','0','base_footprint','base_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0.06 ', '0', '0.00','3.14', '0','0','base_link','laser_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.105', '0', '0.0285','0', '0','0','base_link','camera_link'],),
    ])

    tarkbot_r20_mec = GroupAction([
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='foot_to_base',
            arguments=['0 ', '0', '0.1255','0', '0','0','base_footprint','base_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0 ', '0', '0.06','0', '0','0','base_link','laser_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.105', '0', '0.0285','0', '0','0','base_link','camera_link'],),
    ])
    
    tarkbot_r20_twd = GroupAction([
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='foot_to_base',
            arguments=['0 ', '0', '0.1255','0', '0','0','base_footprint','base_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['0 ', '0', '0.06','0', '0','0','base_link','laser_link'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_camera',
            arguments=['0.105', '0', '0.0285','0', '0','0','base_link','camera_link'],),
    ])

    ld = LaunchDescription()

    ld.add_action(tarkbot_r20_fwd)

    return ld

