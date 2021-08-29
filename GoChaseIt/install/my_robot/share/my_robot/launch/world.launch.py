import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
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

    # spawn robot
    # spawn_robot = Node(
    #     package="robot_spawner_pkg",
    #     executable="spawn_robot",
    #     name="spawn_robot",
    #     remappings=[],
    #     parameters=[{}]
    # )

    urdf = os.path.join(pkg_dir, 'urdf', 'my_robot.xacro')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    swpan_args = '{name: \"my_robot\", xml: \"'  +  xml + '\" }'
    spawn_robot = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen')
    return LaunchDescription([
        gazebo,
        sim_time_process,
        spawn_robot
    ])