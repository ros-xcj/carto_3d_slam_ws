from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    campus_sim_dir = get_package_share_directory('campus_simulation')
    fast_lio_dir = get_package_share_directory('fast_lio')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    # Get the map file path (Your edited 2D map for Nav2 obstacle avoidance)
    map_yaml_file = os.path.join(os.path.expanduser('~'), 'campus_2d_map_4.yaml')

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(campus_sim_dir, 'launch', 'gazebo_advanced.launch.py')
        )
    )

    # 2. FAST_LIO Front-end Odometry (Provides odom -> base_footprint)
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

    # 3. pointcloud_to_laserscan (For Nav2 Local Costmap dynamic avoidance)
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
            'min_height': 0.05,
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
        }]
    )

    # 4. RTAB-Map as a "Silent Observer" (Visual Relocalization -> AMCL)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args': '-r localization_pose:=/initialpose', # CRITICAL: Send calculated pose to AMCL
            'publish_tf_map': 'false', # CRITICAL: Gag RTAB-Map so it DOES NOT publish map->odom TF
            'localization': 'true',    # Pure localization mode
            'frame_id': 'base_footprint',
            'visual_odometry': 'false',
            'odom_topic': '/Odometry',
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/scan_3d',
            'subscribe_scan': 'false',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'rgb_topic': '/camera/camera/image_raw',
            'depth_topic': '/camera/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera/camera_info',
            'approx_sync': 'true',
            'qos': '2',
            'use_sim_time': 'true',
            'rviz': 'false' # Turn off RTAB-Map's Rviz, we only need Nav2's Rviz
        }.items()
    )

    # 5. Nav2 Bringup (Includes AMCL, Map Server, and Navigation)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true',
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
        }.items()
    )

    # 6. Nav2 Rviz2
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    nav2_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_launch,
        fast_lio_launch,
        pctls_node,
        rtabmap_launch,
        nav2_bringup_launch,
        nav2_rviz_node
    ])
