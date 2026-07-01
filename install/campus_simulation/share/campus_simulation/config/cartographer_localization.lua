include "cartographer_3d_2d.lua"

-- 开启纯定位模式，只保留最近的 3 个子图用于定位
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 纯定位时，不需要像建图那样频繁插入新节点，但为了定位准确度，加快全局位姿图优化的频率
POSE_GRAPH.optimize_every_n_nodes = 20

return options
