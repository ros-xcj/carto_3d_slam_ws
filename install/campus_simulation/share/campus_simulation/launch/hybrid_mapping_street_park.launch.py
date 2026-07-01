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
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(campus_sim_dir, 'launch', 'gazebo_street_park.launch.py')
        )
    )

    # 2. FAST_LIO Front-end Odometry
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'velodyne.yaml',
            'use_sim_time': 'true',
            'rviz': 'false'  # Optional: Disable FAST_LIO's Rviz if using RTAB-Map's or your own
        }.items()
    )

    # 3. pointcloud_to_laserscan
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
        }],
        output='screen'
    )

    # 4. RTAB-Map Backend Mapping
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'frame_id': 'base_footprint',
            'visual_odometry': 'false',
            'odom_topic': '/Odometry',
            'subscribe_scan_cloud': 'false',
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'rgb_topic': '/camera/camera/image_raw',
            'depth_topic': '/camera/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera/camera_info',
            'approx_sync': 'true',
            'qos': '2',
            'use_sim_time': 'true',
            'rviz': 'true'
        }.items()
    )

    return LaunchDescription([
        gazebo_launch,
        fast_lio_launch,
        pctls_node,
        rtabmap_launch
    ])
