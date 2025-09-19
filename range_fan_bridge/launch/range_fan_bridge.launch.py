from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('range_fan_bridge')
    config_file = os.path.join(pkg_path, 'config', 'parameter.yaml')  

    return LaunchDescription([
        Node(package='range_fan_bridge',
             executable='range_fan_bridge_node',
             name='range_fan_bridge',
             output='screen',
             parameters=[config_file])
    ])