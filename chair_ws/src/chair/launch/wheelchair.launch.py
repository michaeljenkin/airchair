import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robots = [
        {'name': 'chair_a', 'target': 'burmashave', 'x': '0', 'y': '0', 'theta': '0', 'show_video': 'true', 'camera_tilt': '0.25'} # tilt in radians
        ]
    print(robots)
    urdf = os.path.join(get_package_share_directory('chair'), 'airchair.urdf.xacro')
    print(urdf)


    nodelist = []
    gazebo = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    print(gazebo)
    q = IncludeLaunchDescription(gazebo)
    print(q)
    nodelist.append(q)

    for robot in robots:
        print(robot)
        robot_desc = xacro.process_file(urdf, mappings={'name' : robot['name'], 'target' : robot['target'], 'show_video': robot['show_video'], 'camera_tilt': robot['camera_tilt']}).toxml()
        print(robot_desc)
        nodelist.append(
            Node(
                namespace = robot['name'],
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
                arguments=[urdf])
            )
        nodelist.append(
            Node(
                namespace = robot['name'],
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=["-topic", "/" + robot['name'] + "/robot_description",  "-entity",  robot['name'], 
                           "-x", robot['x'], '-y', robot['y'], '-Y', robot['theta']])
            )
    return LaunchDescription(nodelist)
