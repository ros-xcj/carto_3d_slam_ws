import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('campus_simulation')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer_3d_2d.lua'

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_advanced.launch.py')
        )
    )

    # 2. Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename],
        remappings=[
            # 映射3D雷达话题到points2，这里假设话题名是/scan_3d
            ('points2', '/scan_3d'),
            ('odom', '/odom'),
            ('imu', '/imu')
        ]
    )

    # 3. Occupancy Grid Node (将Cartographer子图转换为2D栅格图)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    # 4. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
