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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

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
            remappings=[
                ('/odom', '/odom_raw'),
                ('/imu', '/imu_raw')
                # Add more remappings as needed
            ],
            )

    foot_to_base = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='foot_to_base',
            arguments=['0 ', '0', '0.125','0', '0','0','base_footprint','base_link'],
            )
            
    joint_pub = launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
    )

    bringup_robot_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_robot')),'launch','robot_description.launch.py'))
    )

    bringup_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_robot')),'launch','ekf.launch.py'))
    )

    bringup_rplidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rplidar_ros')),'launch', 'rplidar_a2m8_launch.py')),
    )    

    bringup_robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('urdf_launch')),'launch','description.launch.py')),
                launch_arguments={
                'urdf_package': 'tarkbot_robot',
                'urdf_package_path': PathJoinSubstitution(['urdf', 'tarkbot_model_v4.urdf'])}.items()
                
    )      

    bringup_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_robot')),'launch','localization_launch.py')),
    )      

    bringup_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('tarkbot_robot')),'launch','navigation_launch.py')),                
    )      


    bringup_foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory(('foxglove_bridge')),'launch','foxglove_bridge_launch.xml')),                
    )   
#     bringup_robot_desc = IncludeLaunchDescription(
#         PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
#         launch_arguments={
#             'urdf_package': 'tarkbot_robot',
#             'urdf_package_path': PathJoinSubstitution(['urdf', 'tarkbot_r20_fwd.urdf'])}.items()
#     )

    ld.add_action(robot_base)
    ld.add_action(foot_to_base)
#     ld.add_action(bringup_robot_description)
#     ld.add_action(bringup_robot_localization)
    ld.add_action(bringup_robot_desc)
    ld.add_action(joint_pub)
    ld.add_action(bringup_rplidar)
    ld.add_action(bringup_amcl)
    ld.add_action(bringup_nav)
    ld.add_action(bringup_foxglove)
    return ld


