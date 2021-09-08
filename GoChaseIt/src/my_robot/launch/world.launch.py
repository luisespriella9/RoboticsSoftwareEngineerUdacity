import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazebo
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_robot = get_package_share_directory('my_robot')
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            )
    )

    urdf = os.path.join(pkg_my_robot, 'urdf', 'my_robot.xacro')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    swpan_args = '{name: \"my_robot\", xml: \"'  +  xml + '\" }'
    spawn_robot = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
            output='screen')

    return LaunchDescription([
            DeclareLaunchArgument(
                'world',
                default_value=[os.path.join(pkg_my_robot, 'worlds', 'empty.world')],
                description='SDF world file'),
            gazebo,
            spawn_robot
    ])