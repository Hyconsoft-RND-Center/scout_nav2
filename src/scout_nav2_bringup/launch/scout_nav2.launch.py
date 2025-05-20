from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='scout_nav2_bringup').find('scout_nav2_bringup')
    default_ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')  

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ekf_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    roas_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roas_bringup'),
                'launch',
                'bringup.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    ouster_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'driver.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen',
        remappings=[
            ('imu/data_raw', '/ouster/imu'),
            ('imu/data', '/imu/data')
        ],
        parameters=[{
            'alpha': 0.98,
            'qos_overrides': {
                '/ouster/imu': {
                    'subscription': {
                        'reliability': 'best_effort',
                        'durability': 'volatile',
                        'history': 'keep_last',
                        'depth': 5
                    }
                }
            }
        }]
    )

    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lio_sam'),
                'launch',
                'lio_sam.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='ekf_config',
            default_value=default_ekf_config_path,
            description='Absolute path to ekf config file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Flag to enable use_sim_time'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=os.path.join(pkg_share, 'rviz', 'config.rviz'),
            description='RViz config path'
        ),

        roas_launch,
        ouster_driver_launch,
        complementary_filter_node,
        lio_sam_launch,
        # robot_localization_node,
        # rviz_node
    ])