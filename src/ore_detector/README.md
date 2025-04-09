# 立方体检测器节点 (Cube Detector Node)

## 概述

这个说明文档是AI写的，我读了一遍基本正确，可供参考

这个 ROS 2 节点 (`cube_detector`) 用于从 2D `sensor_msgs/msg/LaserScan` 消息中检测类似立方体的对象。它估计距离传感器最近的、检测到的立方体面的 2D 位姿（位置和方向）。

核心检测流程包括：
1.  根据视场角 (FOV) 和距离有效性过滤激光扫描点。
2.  将有效点转换为笛卡尔坐标。
3.  使用基于广度优先搜索 (BFS) 的欧几里得聚类算法对邻近点进行聚类。
4.  找到距离传感器最近的聚类。
5.  对最近的聚类进行几何检查，验证其最大跨度是否符合期望的 `cube_size`，以及其点是否表现出高度的线性度（与最远两点连线的偏差小）。这有助于识别正对的立方体面。
6.  如果几何检查通过，使用主成分分析 (PCA) 对聚类点进行处理，以稳健地估计立方体面的方向（角度）。
7.  可选地，应用指数移动平均 (EMA) 滤波器来平滑估计的角度随时间的变化。
8.  发布估计的位姿（`geometry_msgs/msg/PoseStamped`）、用于 RViz 的可视化标记（`visualization_msgs/msg/Marker`）以及一个只包含检测到的立方体点的过滤后的激光扫描（`sensor_msgs/msg/LaserScan`）。

## 工作原理

在 `scanCallback` 函数中执行的处理流程如下：

1.  **预处理:**
    *   遍历输入 `LaserScan` 中的每个点。
    *   过滤掉配置的 `fov_angle` 之外的点。
    *   根据距离有效性（`std::isfinite`，在 `range_min`/`range_max` 内，正值）过滤点。
    *   将有效的极坐标（距离，角度）转换为相对于传感器坐标系的笛卡尔坐标（x, y）。
    *   使用 `PointInfo` 结构体存储笛卡尔坐标点及其在扫描中的原始索引。

2.  **聚类:**
    *   对过滤后的笛卡尔点应用基于 BFS 的欧几里得聚类。
    *   相互距离在 `cluster_threshold` 之内的点被分到同一聚类。
    *   点数少于 `min_points_per_cluster` 的聚类被丢弃。

3.  **寻找最近聚类:**
    *   计算每个有效聚类的质心。
    *   确定哪个聚类的质心距离传感器原点 (0, 0) 最近（欧几里得距离）。

4.  **形状验证（几何检查）:**
    *   对于最近的聚类：
        *   找到聚类中距离最远的两个点。
        *   计算这两点之间的距离 (`max_dist`)。
        *   检查 `max_dist` 是否在 `cube_size +/- size_tolerance` 范围内。
        *   如果长度检查通过，计算聚类中所有其他点到由最远两点定义的**无限长直线**的最大垂直距离 (`max_deviation`)。
        *   检查 `max_deviation` 是否小于 `line_thickness_threshold`（计算为 `cluster_threshold * line_thickness_factor`）。
    *   如果长度和线性度检查都通过，则将 `shape_ok` 标志设置为 true。

5.  **角度计算（PCA）:**
    *   如果 `shape_ok` 为 true：
        *   计算通过验证的聚类中点的协方差矩阵（以聚类质心为中心）。
        *   使用 `Eigen::SelfAdjointEigenSolver` 对协方差矩阵进行特征分解。
        *   提取与*最大*特征值对应的特征向量（这代表第一主成分，即最大方差方向）。
        *   使用这个主特征向量的分量，通过 `std::atan2` 计算 `rotation_angle`。这提供了一个对聚类方向的稳定估计。

6.  **角度平滑（可选）:**
    *   如果 `enable_angle_smoothing` 为 true 且上一步计算出有效角度：
        *   在新计算的 `rotation_angle` 和上一个成功检测周期存储的 `last_valid_angle_` 之间应用 EMA 滤波器，使用 `angle_smoothing_factor`。
        *   使用 `angleDifference` 辅助函数处理角度环绕问题。
        *   更新 `last_valid_angle_`。
    *   如果在任何阶段检测失败，则重置 `last_valid_angle_`。

7.  **发布:**
    *   如果 `shape_ok` 为 true 且 PCA 成功：
        *   在 `/cube_pose` 上发布 `geometry_msgs/msg::PoseStamped` 消息，其位置设置为 `closest_centroid`，方向由（可能平滑过的）`rotation_angle` 导出。
        *   在 `/cube_marker` 上发布 `visualization_msgs/msg/Marker`（CUBE 类型），位于估计的位姿处，用于 RViz 可视化。
        *   在 `/filtered_scan` 上发布 `sensor_msgs/msg/LaserScan` 消息，其中仅包含原始扫描中属于 `closest_cluster` 的点。此消息中的所有其他距离值都设置为无穷大。
    *   如果检测失败，则在 `/filtered_scan` 上发布空的过滤扫描，并且不发布位姿/标记消息。

## 参数

| 参数名                   | 类型   | 默认值 | 描述                                                         |
| :----------------------- | :----- | :----- | :----------------------------------------------------------- |
| `cube_size`              | double | 1.0    | 期望的立方体面边长（米）。                                   |
| `size_tolerance`         | double | 0.2    | 允许的检测跨度的相对偏差 (+/-)。`0.2` 表示 +/- 20%。         |
| `line_thickness_factor`  | double | 5.0    | 乘以 `cluster_threshold` 的因子，定义正对检测时允许的最大厚度。根据传感器噪声和期望的灵敏度与误报率进行调整。 |
| `fov_angle`              | double | 90.0   | 要考虑的传感器的水平视场角（度）。                           |
| `cluster_threshold`      | double | 0.05   | 点属于同一聚类的最大距离（米）。影响分组。                   |
| `min_points_per_cluster` | int    | 10     | 有效聚类所需的最小点数。过滤噪声。                           |
| `enable_angle_smoothing` | bool   | true   | 启用/禁用对输出角度的 EMA 滤波。                             |
| `angle_smoothing_factor` | double | 0.5    | EMA 滤波因子 (0.0-1.0)。值越高表示平滑越强但延迟越大。       |

## 发布的话题

*   `/cube_pose` ([geometry_msgs/msg/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html))
    *   估计的距离传感器最近的立方体面的 2D 位姿。位置是聚类质心，方向通过 PCA 计算（可能经过平滑）。Z 位置以及 X/Y 方向的分量为零。
*   `/cube_marker` ([visualization_msgs/msg/Marker](https://docs.ros2.org/latest/api/visualization_msgs/msg/Marker.html))
    *   一个 CUBE 类型的标记，代表在估计位姿处的检测到的立方体，主要用于 RViz 可视化。
*   `/filtered_scan` ([sensor_msgs/msg/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html))
    *   一个 LaserScan 消息，其中只有属于检测到的立方体聚类的点具有有效的距离值。所有其他距离值都设置为无穷大。有助于可视化哪些点促成了检测。

## 订阅的话题

*   `/hokuyo/lidar` ([sensor_msgs/msg/LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html))
    *   输入的 2D 激光扫描数据。（话题名称可以重映射）。

## 依赖项

*   `rclcpp`
*   `sensor_msgs`
*   `visualization_msgs`
*   `geometry_msgs`
*   `Eigen` (线性代数库 - 通常需要 `eigen3-dev` 或类似的系统包)

## 构建说明

1.  将此功能包放置在您的 ROS 2 工作空间的 `src` 目录下。
2.  导航到您的工作空间的根目录。
3.  使用 colcon 构建：
    ```bash
    colcon build --packages-select <你的功能包名称>
    # 示例: colcon build --packages-select cube_detector_pkg
    ```
4.  Source 工作空间：
    ```bash
    source install/setup.bash
    ```

## 运行节点

```bash
ros2 run <你的功能包名称> cube_detector_node --ros-args -p cube_size:=0.5 -p cluster_threshold:=0.03
# 示例: ros2 run cube_detector_pkg cube_detector_node --ros-args -p cube_size:=0.5
```
您可以使用 `-p <参数名>:=<值>` 语法覆盖任何参数。

## 使用说明与调优

*   **可视化:** 使用 RViz 可视化 `/cube_marker`、`/cube_pose` 和 `/filtered_scan` 话题，并与原始的 `/hokuyo/lidar` 扫描进行对比。这对调优至关重要。确保 RViz 中的 `frame_id` 设置正确（通常是 LaserScan 消息头中指定的坐标系）。
*   **`cube_size` & `size_tolerance`:** 精确设置 `cube_size`。根据传感器噪声或轻微视角变化预期的大小变化来调整 `size_tolerance`。太紧可能漏检，太松可能导致误报。
*   **`cluster_threshold`:** 这个参数很敏感。太小，一个立方体面可能分裂成多个聚类。太大，立方体可能与附近的物体或背景噪声合并。可视化 `/filtered_scan` 来看效果。
*   **`min_points_per_cluster`:** 增加此值以拒绝较小的噪声聚类。如果您的立方体距离较远或扫描稀疏导致有效聚类被拒绝，则减小此值。
*   **`line_thickness_factor`:** 控制聚类需要多“平坦”才能通过几何检查。如果在正对立方体时漏检（检查 DEBUG 日志中的“Linearity failed”），尝试增加此因子。如果检测到非立方体的物体，尝试减小它。
*   **`fov_angle`:** 将处理限制在传感器前方的相关区域。
*   **角度稳定性:** 如果 `/cube_pose` 的方向抖动：
    *   确保 `enable_angle_smoothing` 为 `true`。
    *   增加 `angle_smoothing_factor`（例如，0.7, 0.8）。注意这会增加延迟。
    *   检查 `cluster_threshold` 是否过大，导致聚类边缘的不稳定点对 PCA 产生过大影响。
*   **角点视图:** 当前的几何检查（最大跨度 + 线性度）主要针对相对正对的面进行优化。对于从完美角点观察立方体的情况（点构成 'L' 形），它可能因为线性度检查失败而无法检测。如果需要稳健地检测角点，则需要进行修改以明确处理 L 形聚类。

## 故障排除

*   **没有检测结果:**
    *   检查 RViz：在原始 `/hokuyo/lidar` 扫描的 FOV 内是否可以看到立方体？
    *   检查日志：使用 DEBUG 日志级别运行节点（`--ros-args --log-level debug`）以查看关于滤波、聚类、几何检查（跨度、偏差与阈值对比）和 PCA 结果的详细输出。这通常能 pinpoint 检测失败的原因。
    *   调整参数：放宽 `size_tolerance`，调整 `cluster_threshold`，减小 `min_points_per_cluster`，调整 `line_thickness_factor`。
    *   检查 Frame ID：确保 RViz 和节点在正确的坐标系下运行。
*   **错误检测 (误报):**
    *   收紧 `size_tolerance`。
    *   减小 `line_thickness_factor`。
    *   增加 `min_points_per_cluster`。
    *   调整 `cluster_threshold`。
*   **位姿不稳定:**
    *   启用/调整角度平滑。
    *   检查 `cluster_threshold`。
    *   确保立方体没有快速移动或旋转，超出了平滑滤波器的跟踪能力。