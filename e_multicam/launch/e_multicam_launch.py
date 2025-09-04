#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Width of the camera resolution'
    )
    
    height_arg = DeclareLaunchArgument(
        'height', 
        default_value='1080',
        description='Height of the camera resolution'
    )
    
    num_cam_arg = DeclareLaunchArgument(
        'num_cam',
        default_value='4',
        description='Number of cameras to use'
    )
    
    no_display_arg = DeclareLaunchArgument(
        'no_display',
        default_value='1',
        description='Set no-display streaming to 1'
    )

    no_sync_arg = DeclareLaunchArgument(
        'no_sync',
        default_value='1',
        description='Set no-sync to 1 for disable sync mode'
    )

    # Define the node using ExecuteProcess to avoid ROS2 arguments
    e_multicam_node = ExecuteProcess(
        cmd=[
            '/mnt/bd8b23c6-4c6c-4a1c-a460-08f3112b46bc/camera_test_ws/install/e_multicam/lib/e_multicam/e_multicam',
            '--width', LaunchConfiguration('width'),
            '--height', LaunchConfiguration('height'),
            '--num_cam', LaunchConfiguration('num_cam'),
            '--no-display', LaunchConfiguration('no_display'),
            '--no-sync', LaunchConfiguration('no_sync')
        ],
        output='screen',
        name='e_multicam_node'
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        num_cam_arg,
        no_display_arg,
        no_sync_arg,
        e_multicam_node
    ]) 
