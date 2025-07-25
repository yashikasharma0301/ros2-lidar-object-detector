import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
import xacro

def generate_launch_description():
  
    pkg_name = 'object_distance'
    xacro_file_name = 'myrobot.xacro'
    world_file_name='demo_world.sdf'
  
    xacro_file = os.path.join(
    get_package_share_directory(pkg_name),
    'urdf',
    xacro_file_name)
  
    world_file = os.path.join(
    get_package_share_directory(pkg_name),
    'worlds',
    world_file_name)

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] )
 
    gazebo = ExecuteProcess(
    cmd=['ign', 'gazebo', world_file, '--verbose', '-r'],
    output='screen')

    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'myrobot',
        '-topic', 'robot_description',
        '-x', '0', '-y', '0', '-z', '0.335' 
    ],
    output='screen',)
    
    bridge_params = os.path.join(
    get_package_share_directory(pkg_name),
    'params',
    'myrobot.yaml')

    start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}',
    ],
    output='screen',)
   
    return LaunchDescription([
        gazebo,
        start_gazebo_ros_bridge_cmd,
        node_robot_state_publisher,
        spawn_entity
    ])
