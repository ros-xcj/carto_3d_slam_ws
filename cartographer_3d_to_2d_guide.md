# Cartographer 3D雷达建2D导航图配置指南 (适配Nav2)

本文档基于您的 `carto_slam_ws` 工作空间环境（ROS 2）以及您提供的参考博客内容，为您整理了**使用3D激光雷达直接构建2D栅格地图（供Nav2导航使用）**的详细配置指南和原理解析。

## 1. 原理分析 (结合参考博客)

在使用3D激光雷达进行建图时，如果最终目的是为了给机器人平面导航（Nav2）使用，通常会面临两种选择：
1. **纯3D建图后投影2D**：计算量大，且如果轨迹出现轻微闭环误差，投影到2D地图上会出现“重影”或明显的“雷达波纹”，地图不干净（如同博客中提到的问题）。
2. **直接使用3D数据进行2D建图 (推荐方案)**：利用 Cartographer 的 `use_trajectory_builder_2d = true`，并将输入的3D点云数据 (`num_point_clouds = 1`) 截取特定高度 `[min_z, max_z]` 进行2D匹配。
    - **优势**：大幅减少计算量，生成的2D概率栅格地图干净且无“波纹”，直接完美契合Nav2的需求。

---

## 2. 核心配置文件修改

在ROS 2中，我们需要重点配置 Cartographer 的 `.lua` 文件以及编写相应的 Launch 文件。假设您在 `campus_simulation` 或自定义的包中配置：

### 2.1 Cartographer Lua 参数配置 (`cartographer_3d_2d.lua`)

创建一个lua配置文件（例如在 `campus_simulation/config/` 下）。核心思路是：**关闭LaserScan，开启PointCloud2输入，开启2D建图，并设置Z轴过滤**。

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",       -- 或 "imu_link" (如果使用IMU且IMU频率最高)
  published_frame = "base_link",      -- 根据TF树设置，通常是 base_link 或 odom
  odom_frame = "odom",
  provide_odom_frame = true,          -- 如果外部不提供odom，这里设为true让Carto发布
  publish_frame_projected_to_2d = true, -- 对2D建图很重要，防止TF出现Z轴偏移
  use_odometry = true,                -- 是否使用外部里程计 (根据实际情况设置)
  use_nav_sat = false,
  use_landmarks = false,
  
  -- 【核心修改1：传感器输入设置】
  num_laser_scans = 0,                -- 关闭单线雷达输入
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,               -- 开启3D点云输入！
  
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 【核心修改2：开启2D建图而非3D】
MAP_BUILDER.use_trajectory_builder_2d = true

-- 【核心修改3：2D轨迹构建器对于3D点云的裁剪】
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 30.0   -- 根据您的3D雷达实际范围调整
TRAJECTORY_BUILDER_2D.min_z = 0.1        -- 过滤地面点！重要！(低于0.1m的点忽略)
TRAJECTORY_BUILDER_2D.max_z = 1.5        -- 过滤过高的点！(高于1.5m的点忽略，防止天花板/树冠建入障碍物)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.

-- 【核心修改4：若里程计较差或不使用IMU，建议开启在线相关性扫描匹配】
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 如果没有稳定IMU，建议设为false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 调整后端优化频率
POSE_GRAPH.optimize_every_n_nodes = 35

return options
```

### 2.2 ROS 2 Launch 文件配置 (`cartographer_mapping.launch.py`)

编写ROS 2 launch文件来启动 Cartographer 节点并加载上述配置，同时必须启动 `occupancy_grid_node` 来实时发布二维栅格图。

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('campus_simulation') # 替换为您的包名
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer_3d_2d.lua'

    # Cartographer核心节点
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
            # 【重要】将您的3D雷达点云话题映射到 points2！
            ('points2', '/velodyne_points'), # 根据您的实际话题修改，如 /livox/lidar 等
            ('odom', '/odom'),
            ('imu', '/imu/data')
        ]
    )

    # Occupancy Grid节点：负责将Cartographer的子图转换为Nav2可用的2D栅格地图
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        cartographer_node,
        occupancy_grid_node,
    ])
```

---

## 3. 从建图到导航的测试流程

### 第一步：启动建图
1. 启动您的机器人仿真 (Gazebo) 或者真实机器人底层及雷达驱动。
2. 启动上面的 Cartographer 建图 Launch 文件：
   ```bash
   ros2 launch campus_simulation cartographer_mapping.launch.py
   ```
3. 启动 RViz2 进行可视化：
   - 添加 `Map` 插件，订阅 `/map` 话题。
   - 遥控机器人 (`teleop_twist_keyboard` 或手柄) 在环境中游走。
   - 您应该能看到清晰的 2D 栅格地图（障碍物为黑色，可通行区域为白色，未知区域为灰色）正在构建。

### 第二步：保存地图 (Nav2 格式)
在 ROS 2 中，我们使用 `nav2_map_server` 提供的工具来保存地图。在地图构建完整后，新开一个终端运行：

```bash
# 确保安装了 nav2_map_server
ros2 run nav2_map_server map_saver_cli -f ~/qirui/robot/carto_slam_ws/src/campus_simulation/maps/my_2d_map
```
这将在指定路径下生成两个文件，完美匹配 Nav2 的要求：
- `my_2d_map.pgm` (图像文件)
- `my_2d_map.yaml` (地图描述文件)

### 第三步：配置并启动 Nav2 进行导航测试
在您之前工作的 `/home/zzl/qirui/robot/hybrid_slam_ws/src/campus_simulation/config/nav2_params.yaml` 配置文件中，主要关注点是代价地图 (`costmap`)。由于我们生成了高质量的2D栅格图，Nav2的全局和局部规划将非常顺利。

1. **修改 Nav2 Launch 或参数**，将 `map_server` 的地图指向刚刚保存的地图：
   在您的 Navigation 启动文件中，确保 `map` 参数指向 `my_2d_map.yaml`。
   
2. **启动 Nav2 栈**：
   ```bash
   ros2 launch campus_simulation hybrid_navigation_rtabmap.launch.py map:=/home/zzl/qirui/robot/carto_slam_ws/src/campus_simulation/maps/my_2d_map.yaml
   ```
   *(根据您实际的launch命令进行调整)*

3. **在 RViz 中测试**：
   - 使用 `2D Pose Estimate` 工具给出机器人的初始位姿（AMCL初始化）。
   - 使用 `Nav2 Goal` (2D Goal Pose) 给定目标点。
   - 观察机器人是否能够顺滑地避开障碍物并到达目标点。

## 4. 常见问题排查 (Troubleshooting)

1. **地图上没有白色区域（只有黑灰）或有很多噪点？**
   - 检查 `.lua` 文件中的 `TRAJECTORY_BUILDER_2D.min_z` 和 `max_z`。通常地面点未滤除干净会导致此现象，适当调高 `min_z` (例如 `0.1` 或 `0.15`) 过滤地面。
   
2. **建图过程中出现轨迹跳动或地图重影？**
   - 如果不使用轮式里程计（`use_odometry = false`），请务必确保 `use_online_correlative_scan_matching = true`。
   - 如果使用里程计，请检查 `/odom` 话题的精度和协方差。

3. **3D雷达数据量太大导致建图卡顿？**
   - 可以通过 `voxel_filter_size` 对点云进行下采样，在 `.lua` 中添加：`TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05`。
