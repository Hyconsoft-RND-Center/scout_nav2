from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import tempfile
import subprocess

def generate_launch_description():
    pkg_share = FindPackageShare(package='scout_bot_description').find('scout_bot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    default_model_path = os.path.join(pkg_share, 'description', 'roas2_gazebo.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    tmp_sdf = tempfile.NamedTemporaryFile(delete=False, suffix='.sdf').name

    # Robot State Publisher : URDF를 읽고 TF를 퍼블리시
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    {'use_sim_time': True}
                    ]
    )
    

    # sdf 파일6 변환을 위한 임시 sdf 파일 생성
    try:
        sdf_str = subprocess.check_output(['gz', 'sdf', '-p', default_model_path])
    except subprocess.CalledProcessError as e:
        print("[Launch Error] URDF to SDF 변환 실패:", e.output.decode())
        raise
    tmp_sdf_file = tempfile.NamedTemporaryFile(delete=False, suffix='.sdf', mode='w')
    tmp_sdf_file.write(sdf_str.decode())
    tmp_sdf_file.close()
    sdf_path = tmp_sdf_file.name

    # Joint State Publisher : 여기서는 기본 모델 파일 경로만 인자로 전달
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
        parameters=[{'use_sim_time': True}]
    )
    
    # Gazebo 실행 (ROS2 패키지 형식)
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'scout_bot', '-file', sdf_path,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.5',
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', '0.0'],
        output='screen'
    )

    spawn_walls = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # arguments=['-entity', 'test_wall', 
        #            '-file', test_wall_path]
        output='screen'
    )

    # robot_localization의 ekf_node를 실행하여 센서 데이터를 융합하고 transform을 게시함
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'),
                    #{'use_sim_time': LaunchConfiguration('use_sim_time')}
                    {'use_sim_time': True}
                    ]
    )

    # RViz 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'),
                   ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        DeclareLaunchArgument('gui', default_value='false',
            description='Disable gzclient GUI'),
        DeclareLaunchArgument('server', default_value='true',
            description='Enable gzserver'),

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f"xacro {default_model_path} | gz sdf -p > {tmp_sdf}"
            ],
            shell=True
        ),

        # gazebo_process,
        gazebo,
        joint_state_publisher_node, 
        robot_state_publisher_node,
        spawn_entity, # 로봇 생성
        # spawn_walls,
        robot_localization_node,
        rviz_node
    ])
