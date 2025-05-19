from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='scout_nav2_bringup').find('scout_nav2_bringup')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    default_localization_config_path = os.path.join(pkg_share, 'config', 'localization.yaml')
    default_nav2_config_path = os.path.join(pkg_share, 'config', 'nav2_param.yaml')
    default_ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.ymal')
    
    # robot_localization의 ekf_node를 실행하여 센서 데이터를 융합하고 transform을 게시함
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=['ekf_config',
                    #{'use_sim_time': LaunchConfiguration('use_sim_time')}
                    {'use_sim_time': 'use_sim_time'}
                    ]
    )

    # lidar_localization_ros2
    lidar_localization_node = LifecycleNode(
        name='lidar_localization',
        package='lidar_localization_ros2',
        namespace='',
        executable='lidar_localization_node',
        parameters=[default_localization_config_path, {'use_sim_time': 'use_sim_time'}],
        remappings=[('/velodyne_points', '/points_raw'),
                    ('/odom', '/odom'),
                    ('/imu', '/imu')],
        output='screen'
    )

    # nav2 실행
    nav2_node = Node(
        packages='nav2_bringup',
        executable='nav2_bringup',
        name='nav2_bringup',
        output='screen',
        parameters=[{'use_sim_time': 'use_sim_time'},
                    {'params_file': 'nav2_config'}]
    )

    
    # RViz 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'),
                   {'use_sim_time': 'use_sim_time'}
                   ]
    )

    

    return LaunchDescription([
        # launch 인자 선언: 로봇 모델, RViz 설정, 그리고 use_sim_time
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='nav2_config',
            default_value=default_nav2_config_path,
            description='Absolute path to nav2 config file'
        ),
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
        
        lidar_localization_node,
        nav2_node,
        #robot_localization_node,
        rviz_node
    ])
