import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.substitutions import Command      #Command在Dashing中识别不了
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    tarkbot_config = os.path.join(get_package_share_directory('tarkbot_robot'),'config','tarkbot_config.yaml')

    with open(tarkbot_config,'r') as f:
        params = yaml.safe_load(f)["tarkbot_robot_node"]["ros__parameters"]

    robot_base = Node(
            package= "tarkbot_robot",                 #功能包。
            executable= "tarkbot_robot_node",         #节点。
            parameters= [tarkbot_config],             #接入参数文件
            output= 'screen',
            )

    base_to_imu = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_imu',
            arguments=['0', '0', '0','0', '0','0','base_link','imu_link'],
            output= 'screen',
            
    )

#     bringup_robot_description = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_robot')),'launch','robot_description.launch.py'))
#     )
    ld.add_action(robot_base)
    ld.add_action(base_to_imu)
#     ld.add_action(bringup_robot_description)

    return ld


