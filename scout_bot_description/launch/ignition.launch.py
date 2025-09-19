import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_scout_bot_description = get_package_share_directory('scout_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    ign_model_path = os.path.expanduser('~/.ignition/fuel/fuel.gazebosim.org/')

    # URDF 파일 경로 설정
    urdf_file = os.path.join(pkg_scout_bot_description, 'description', 'roas2_ignition.xacro')

    controllers_yaml_path = os.path.join(pkg_scout_bot_description, 'config', 'gazebo_controllers.yaml')

    # XACRO를 사용하여 URDF를 XML 문자열로 파싱
    robot_description_str = xacro.process_file(urdf_file, 
                                               mappings={'controllers_yaml_path': controllers_yaml_path}).toxml()

    # Launch 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 플러그인 포함 World 역할 파일
    world_file = os.path.join(pkg_scout_bot_description, 'worlds', 'empty.sdf')
    gz_args = f"-r {world_file}"
    
    # Ignition Gazebo가 모델(메시) 파일을 찾을 수 있도록 환경 변수 설정
    # 설치된 패키지의 share 디렉토리를 GZ_SIM_RESOURCE_PATH에 추가
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=":".join([
            os.path.join(pkg_scout_bot_description, '..'),
            os.path.join(pkg_scout_bot_description, 'models'),
            # 다른 패키지 경로도 필요 시 추가
        ])
    )
    # Gazebo가 ROS 2로 설치된 플러그인을 찾을 수 있도록 경로 설정
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(get_package_share_directory('ros_gz_bridge'), 'lib'),
            os.path.join(get_package_share_directory('ign_ros2_control'), 'lib'),
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    # ros_gz_bridge yaml 파일 경로
    bridge_yaml_path = os.path.join(pkg_scout_bot_description, 'config', 'scout_bridge.yaml')

    # Ignition Gazebo 실행
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    ) 

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_str},
                    {'use_sim_time': use_sim_time}]
    )
    
    # Spawn Wall
    # wall_model_path = os.path.join(pkg_scout_bot_description, 'models', 'lobby', 'model.sdf')
    # wall_model_path = os.path.join(pkg_scout_bot_description, 'models', 'new_lobby', 'model.sdf')
    wall_model_path = os.path.join(ign_model_path, 'openrobotics', 'models', 'baylands', '3', 'model.sdf')
    spawn_wall = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wall',
            '-file', os.path.join(pkg_scout_bot_description, wall_model_path),
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '3.0' # 3.0
        ],
        output='screen'
    )

    # Gazebo에 로봇 스폰
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[ 
            '-string', robot_description_str,
            '-name', 'scout_bot',
            '-allow_renaming', 'true',
            '-x', '-200.0', # -200.0
            '-y', '-170.0', # -170.0
            '-z', '6.66' # base_link 기준이므로 높이 0에서 소환하는 경우 타이어가 땅에 박힐 수 있음
        ],
        output='screen'
    )
    
    # ROS <=> Gazebo bridge
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}],
        output='screen'
    )

    # ROS <=> Gazebo Image Bridge
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        parameters=[],
        arguments=[
                '/Cam1_front_camera/image_raw',
                '/Cam2_right_camera/image_raw',
                '/Cam3_left_camera/image_raw',
                '/Cam4_rear_camera/image_raw'
                ],
        output='screen'
    )

    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     remappings=[('cloud_in', '/lidar'),
    #                 ('scan_out', '/scan')],
    #     parameters=[{
    #         'target_frame': 'os_sensor',
    #         'min_height': -0.1, 
    #         'max_height': 0.1, 
    #         'angle_min': -3.14159,
    #         'angle_max': 3.14159,
    #         'range_min': 0.3,
    #         'range_max': 50.0,
    #         'use_sim_time': use_sim_time
    #     }]
    # )

       # ros2_control: controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name='controller_manager',  
        parameters=[
            {'robot_description': robot_description_str},
            os.path.join(pkg_scout_bot_description, 'config', 'gazebo_controllers.yaml')
        ],
        output="screen",
    )
    # Spawn joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--switch-timeout", "30"], 
        output="screen",
    )

    # scout_drive_controller
    scout_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scout_drive_controller", "--controller-manager", "/controller_manager",
                   "--switch-timeout", "30"],
        output="screen",
    )

    # EKF 
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_scout_bot_description, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}]
    )
    
    # NavSat(GNSS)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_scout_bot_description, 'config', 'navsat.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings = [('/gps/fix', '/fix') ]
    )
    
    # RViz 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_scout_bot_description, 'rviz', 'config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    marker_publisher_node = Node(
        package='navi_plugin',
        executable='waypoint_marker_publisher',
        name='waypoint_marker_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    pose_marker_publisher_node = Node(
        package='navi_plugin',
        executable='pose_marker_publisher',
        name='pose_marker_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    return LaunchDescription([
        set_gz_resource_path,
        set_gz_plugin_path,
        gz_sim,
        robot_state_publisher_node,
        spawn_wall,
        spawn_robot,
        gz_ros_bridge,
        image_bridge,
        #pointcloud_to_laserscan_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        scout_drive_controller,
        robot_localization_node,
        navsat_transform_node,
        rviz_node,
        # marker_publisher_node,
        pose_marker_publisher_node,
    ])
