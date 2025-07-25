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
  
    pkg_name = 'tortoisebot_description'
    this_pkg = 'tortoisebot_gazebo'
    file_subpath = 'urdf/tortoisebot.xacro'
    world_file_name='demo_world.sdf'
    
    world_path = os.path.join(
    get_package_share_directory(this_pkg),
    'worlds',
    world_file_name)


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    # Launch Gazebo
 
    gazebo = ExecuteProcess(
    cmd=['ign', 'gazebo', world_path, '--verbose', '-r'],
    output='screen'
)


    spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'tortoisebot',
        '-topic', 'robot_description',
        '-x', '0', '-y', '0', '-z', '0.335' 
    ],
    output='screen',)
    
    bridge_params = os.path.join(
    get_package_share_directory('tortoisebot_gazebo'),
    'params',
    'tortoisebot.yaml')

    start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}',
    ],
    output='screen',)
   
    # Run the node
    return LaunchDescription([
        start_gazebo_ros_bridge_cmd,
        node_robot_state_publisher,
        spawn_entity,
        gazebo
    ])
