import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('campus_simulation')
    
    world_file_name = 'indoor_outdoor.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    urdf_file_name = 'advanced_robot.urdf.xacro'
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    
    doc = xacro.process_file(urdf_path)
    robot_description = {'robot_description': doc.toxml()}
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description,{'use_sim_time': True}]
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'advanced_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
