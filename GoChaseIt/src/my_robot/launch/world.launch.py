import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Gazebo
    world_file_name = 'empty.world'
    pkg_dir = get_package_share_directory('my_robot')
    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world],
            output='screen')

    # # Sim time
    sim_time_process = ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen')
    # DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value=use_sim_time,
    #         description='Use simulation (Gazebo) clock if true')

    # spawn robot
    spawn_robot = Node(
            package='robot_spawner_pkg',
            executable='spawn_robot',
            parameters=['the_robot_name', 'robot_namespace', '0.0', '0.0', '0.1'],
            output='screen')
    return LaunchDescription([
        gazebo,
        sim_time_process,
        spawn_robot
    ])