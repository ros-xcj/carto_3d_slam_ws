from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    campus_sim_dir = get_package_share_directory('campus_simulation')
    fast_lio_dir = get_package_share_directory('fast_lio')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get the map file path (Change this if you move the map file)
    map_yaml_file = os.path.join(os.path.expanduser('~'), 'campus_2d_map_4.yaml')

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(campus_sim_dir, 'launch', 'gazebo_advanced.launch.py')
        )
    )

    # 2. FAST_LIO Front-end Odometry (Providing high quality odom -> base_footprint)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'velodyne.yaml',
            'use_sim_time': 'true',
            'rviz': 'false'
        }.items()
    )

    # 3. pointcloud_to_laserscan (Crucial for AMCL and Nav2 Costmaps)
    pctls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/scan_3d'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'laser_link',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': True
        }],
        output='screen'
    )

    # 4. Nav2 Bringup (AMCL + Planning + Control)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'params_file': os.path.join(campus_sim_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # 5. Rviz2 for Nav2
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        fast_lio_launch,
        pctls_node,
        nav2_launch,
        rviz_node
    ])
