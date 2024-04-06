from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os




def generate_launch_description():
    # Path to config file
    elevator_config = os.path.join(get_package_share_directory('multi_floor_navigator'),'config','multi_floor_config.yaml')

    

    # Node to run the custom node
    multi_floor_goal = Node(
        package='multi_floor_navigator',
        executable='multi_floor_goal',
        output='screen',
        parameters=[elevator_config],
    )

    return LaunchDescription([
        multi_floor_goal
    ])
