# 文件名：gazebo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable
from launch.substitutions import Command

def generate_launch_description():
    # 参数配置
    package_name = 'wuthering_waves'  
    world_file_name = 'RMUC2024_world.world'    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 路径获取
    pkg_path = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_path, 'urdf', 'slam_bot.xacro')  
    world_path = os.path.join(pkg_path, 'worlds', world_file_name)

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),  
            ' ',
            urdf_path
        ]),
        value_type=str  
    )

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,  
            'use_sim_time': use_sim_time
        }]
    )
    
    # 2. 启动Gazebo服务
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'pause': 'false',
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 3. 生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wuthering_waves_robot',
            '-topic', 'robot_description',
            '-x', '5.0',
            '-y', '8.0',
            '-z', '0.4',  
            '-timeout', '6'  
        ],
        output='screen'
    )
        

    return LaunchDescription([
        start_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])