#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <limits> // 需要 std::numeric_limits
#include <optional> // 需要 std::optional

// 结构体：存储点数据及其在原始扫描中的索引
struct PointInfo {
    size_t index;       // 在 LaserScan ranges 数组中的原始索引
    Eigen::Vector2d point; // 相对于传感器的笛卡尔坐标 (x, y)
};

/**
 * @brief 从 2D LaserScan 数据中检测类似立方体的对象。
 *
 * 此节点订阅 LaserScan 消息，过滤指定视场角 (FOV) 内的点，
 * 对邻近点进行聚类，找到最近的聚类，验证该聚类的几何形状
 * (跨度与线性度) 是否匹配立方体的面，使用聚类的质心和主成分分析 (PCA)
 * 稳定地估计姿态 (位置和方向)，并发布姿态、可视化标记以及
 * 一个只包含检测到的立方体点的过滤后的扫描。包含可选的角度平滑功能。
 */
class CubeDetector : public rclcpp::Node {
public:
    CubeDetector() : Node("cube_detector") {
        // --- 参数声明与初始化 ---
        // 注意：这里设置了默认值，但可以通过 ROS 参数覆盖。

        // 期望的立方体边长（米）。
        this->declare_parameter("cube_size", 1.0);
        cube_size_ = this->get_parameter("cube_size").as_double();

        // 匹配立方体尺寸的容差（以小数表示）。
        // 例如，0.2 表示检测到的跨度必须在 [cube_size * 0.8, cube_size * 1.2] 范围内。
        this->declare_parameter("size_tolerance", 0.2);
        size_tolerance_ = this->get_parameter("size_tolerance").as_double();

        // 乘以 cluster_threshold 的因子，用于确定允许的最大“厚度”
        // （点到最远两点连线的最大偏差），以便将聚类视为“线状”
        // （即，正对着一个面）。
        // 较大的值对较粗的线/噪声更宽容，但可能增加误报。
        this->declare_parameter("line_thickness_factor", 5.0);
        line_thickness_factor_ = this->get_parameter("line_thickness_factor").as_double();

        // 传感器的水平视场角 (FOV)，单位为度。此角度范围之外的点
        // （以传感器前方为中心）将被忽略。
        this->declare_parameter("fov_angle", 90.0);
        fov_angle_ = this->get_parameter("fov_angle").as_double();

        // 欧几里得聚类时，两个点被视为同一聚类的最大距离（米）。
        // 影响点的分组方式；较小的值可能分割物体，较大的值可能合并邻近物体。
        this->declare_parameter("cluster_threshold", 0.05);
        cluster_threshold_ = this->get_parameter("cluster_threshold").as_double();

        // 一个聚类被视为有效所需的最小点数。
        // 有助于过滤掉小的噪声聚类。
        this->declare_parameter("min_points_per_cluster", 10);
        min_points_per_cluster_ = this->get_parameter("min_points_per_cluster").as_int();

        // 启用/禁用对估计角度的指数移动平均 (EMA) 滤波。
        // 用于减少输出方向的抖动。
        this->declare_parameter("enable_angle_smoothing", true);
        enable_angle_smoothing_ = this->get_parameter("enable_angle_smoothing").as_bool();

        // EMA 滤波器的平滑因子 (0.0 到 1.0)。
        // 0.0 = 无平滑（直接使用当前角度）。
        // 1.0 = 最大平滑（角度从第一个有效值开始不再改变）。
        // 值越接近 1.0，平滑效果越强，但延迟越大。典型值：0.5 - 0.9。
        this->declare_parameter("angle_smoothing_factor", 0.5);
        angle_smoothing_factor_ = this->get_parameter("angle_smoothing_factor").as_double();

        // --- ROS 通信设置 ---
        // 订阅输入的 LaserScan 数据
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/hokuyo/lidar", // 话题名称 (可以重映射)
            10,              // QoS 历史深度
            std::bind(&CubeDetector::scanCallback, this, std::placeholders::_1));

        // 发布估计的立方体姿态
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_pose", 10);
        // 发布可视化标记 (用于 RViz)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cube_marker", 10);
        // 发布过滤后的扫描 (只显示立方体的点)
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);


        // --- 初始化日志 ---
        RCLCPP_INFO(this->get_logger(), "立方体检测器已初始化:");
        RCLCPP_INFO(this->get_logger(), "  立方体尺寸: %.2f 米", cube_size_);
        RCLCPP_INFO(this->get_logger(), "  尺寸容差: +/- %.1f %%", size_tolerance_ * 100.0);
        RCLCPP_INFO(this->get_logger(), "  线厚度因子: %.1f", line_thickness_factor_);
        RCLCPP_INFO(this->get_logger(), "  视场角 (FOV): %.1f 度", fov_angle_);
        RCLCPP_INFO(this->get_logger(), "  聚类阈值: %.3f 米", cluster_threshold_);
        RCLCPP_INFO(this->get_logger(), "  每簇最小点数: %d", min_points_per_cluster_);
        RCLCPP_INFO(this->get_logger(), "  角度平滑已启用: %s", enable_angle_smoothing_ ? "是" : "否");
        if (enable_angle_smoothing_) {
            RCLCPP_INFO(this->get_logger(), "  角度平滑因子: %.2f", angle_smoothing_factor_);
        }
    }

private:
    // --- 成员变量 ---
    double cube_size_;
    double size_tolerance_;
    double line_thickness_factor_;
    double fov_angle_;
    double cluster_threshold_;
    int min_points_per_cluster_;
    bool enable_angle_smoothing_;
    double angle_smoothing_factor_;

    // 用于角度平滑的状态变量
    std::optional<double> last_valid_angle_ = std::nullopt;

    // ROS 发布者和订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;

    // --- 辅助函数 ---

    /**
     * @brief 计算两个角度之间的最短差值，处理在 -PI/PI 处的环绕。
     * @param angle1 第一个角度（弧度）。
     * @param angle2 第二个角度（弧度）。
     * @return 差值 angle1 - angle2，归一化到 [-PI, PI]。
     */
    double angleDifference(double angle1, double angle2) {
        double diff = angle1 - angle2;
        while (diff <= -M_PI) diff += 2.0 * M_PI;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        return diff;
    }

    // --- 主要回调函数 ---

    /**
     * @brief 处理传入的 LaserScan 消息以检测立方体。
     * @param scan_msg 收到的 LaserScan 消息的共享指针。
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // === 阶段 1: 预处理 (FOV滤波, 有效性检查, 坐标转换) ===
        std::vector<PointInfo> points_with_info;
        double half_fov_rad = fov_angle_ / 2.0 * M_PI / 180.0;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // 应用FOV滤波
            if (angle < -half_fov_rad || angle > half_fov_rad) continue;

            double range = scan_msg->ranges[i];
            // 检查距离测量的有效性
            if (!std::isfinite(range) || range <= scan_msg->range_min || range >= scan_msg->range_max || range <= 1e-3) { // 添加小的正值检查
                 continue;
             }

            // 将有效的极坐标转换为笛卡尔坐标
            double x = range * cos(angle);
            double y = range * sin(angle);
            points_with_info.push_back({i, Eigen::Vector2d(x, y)});
        }

        // 检查滤波后是否还有足够的点
        if (points_with_info.size() < min_points_per_cluster_) {
            RCLCPP_DEBUG(this->get_logger(), "FOV内有效点不足: %zu", points_with_info.size());
            publishFilteredScan({}, scan_msg); // 发布空扫描
            if (enable_angle_smoothing_) { last_valid_angle_ = std::nullopt; } // 重置平滑状态
            return;
        }

        // === 阶段 2: 聚类 (BFS 欧几里得聚类) ===
        std::vector<std::vector<PointInfo>> clusters;
        std::vector<bool> visited(points_with_info.size(), false);

        for (size_t i = 0; i < points_with_info.size(); ++i) {
            if (visited[i]) continue;

            std::vector<PointInfo> current_cluster;
            std::vector<size_t> queue; // 队列存储 points_with_info 中的索引
            queue.push_back(i);
            visited[i] = true;

            int head = 0;
            while(head < queue.size()) {
                // 防止队列异常增大的安全中断
                if (queue.size() > points_with_info.size()) {
                     RCLCPP_WARN(this->get_logger(), "聚类队列异常增大 (%zu > %zu)，中断。", queue.size(), points_with_info.size());
                     current_cluster.clear();
                     // 将队列中剩余点标记为已访问，避免重复处理
                     for(size_t q_idx = head; q_idx < queue.size(); ++q_idx) {
                         if (queue[q_idx] < visited.size()) visited[queue[q_idx]] = true;
                     }
                     break;
                 }

                size_t current_point_q_idx = queue[head++];
                current_cluster.push_back(points_with_info[current_point_q_idx]);

                // 查找 cluster_threshold 范围内的邻居
                for (size_t j = 0; j < points_with_info.size(); ++j) {
                    if (visited[j] || current_point_q_idx == j) continue;

                    double dist_sq = (points_with_info[current_point_q_idx].point - points_with_info[j].point).squaredNorm();
                    if (dist_sq <= cluster_threshold_ * cluster_threshold_) {
                        visited[j] = true;
                        queue.push_back(j);
                    }
                }
            }

            // 如果聚类满足最小点数要求，则保留
            if (!current_cluster.empty() && current_cluster.size() >= min_points_per_cluster_) {
                clusters.push_back(current_cluster);
            }
        }

        if (clusters.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "聚类后未找到有效聚类。");
            publishFilteredScan({}, scan_msg);
            if (enable_angle_smoothing_) { last_valid_angle_ = std::nullopt; } // 重置平滑状态
            return;
        }

        // === 阶段 3: 找到最近的聚类 ===
        int closest_cluster_idx = -1;
        double min_centroid_dist_sq = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_centroid(0,0);

        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& cluster = clusters[i];
            // 此检查在聚类后检查有点多余，但为了安全起见加上。
            if (cluster.size() < min_points_per_cluster_) continue;

            // 计算聚类质心
            Eigen::Vector2d centroid(0.0, 0.0);
            for (const auto& p_info : cluster) {
                centroid += p_info.point;
            }
            centroid /= static_cast<double>(cluster.size());

            // 找到质心离传感器（原点）最近的聚类
            double dist_sq = centroid.squaredNorm();
            if (dist_sq < min_centroid_dist_sq) {
                min_centroid_dist_sq = dist_sq;
                closest_cluster_idx = i;
                closest_centroid = centroid;
            }
        }

        if (closest_cluster_idx == -1) {
            // 理论上如果 clusters 列表不为空，这不应发生，但做防御性处理。
            RCLCPP_DEBUG(this->get_logger(), "在 %zu 个候选聚类中未能确定最近的聚类。", clusters.size());
            publishFilteredScan({}, scan_msg);
            if (enable_angle_smoothing_) { last_valid_angle_ = std::nullopt; } // 重置平滑状态
            return;
        }

        // === 阶段 4: 验证最近聚类的形状 (几何检查: 跨度与线性度) ===
        const auto& closest_cluster = clusters[closest_cluster_idx];
        bool shape_ok = false;

        // 至少需要2个点来确定跨度和线性度
        if (closest_cluster.size() >= 2) {
            // 4.1 找到聚类中距离最远的两个点（确定最大跨度）
            double max_dist_sq = -1.0;
            size_t idx1 = 0, idx2 = 0;
            // 简单的 O(N^2) 搜索 - 对于典型的聚类大小通常可以接受
            for (size_t i = 0; i < closest_cluster.size(); ++i) {
                for (size_t j = i + 1; j < closest_cluster.size(); ++j) {
                    double dist_sq = (closest_cluster[i].point - closest_cluster[j].point).squaredNorm();
                    if (dist_sq > max_dist_sq) {
                        max_dist_sq = dist_sq;
                        idx1 = i;
                        idx2 = j;
                    }
                }
            }

            // 确保找到了有效的最远点
            if (max_dist_sq > 1e-9) { // 使用小 epsilon 增加鲁棒性
                double max_dist = std::sqrt(max_dist_sq);
                const Eigen::Vector2d p1 = closest_cluster[idx1].point;
                const Eigen::Vector2d p2 = closest_cluster[idx2].point;

                // 4.2 检查最大跨度是否在容差范围内符合期望的立方体尺寸
                double lower_bound = cube_size_ * (1.0 - size_tolerance_);
                double upper_bound = cube_size_ * (1.0 + size_tolerance_);
                bool length_ok = (max_dist >= lower_bound && max_dist <= upper_bound);

                RCLCPP_DEBUG(this->get_logger(), "聚类最大跨度: %.3f 米. 要求: [%.3f, %.3f]",
                            max_dist, lower_bound, upper_bound);

                if (length_ok) {
                    // 4.3 检查聚类的线性度：点偏离 p1-p2 定义的直线的程度
                    double max_deviation_sq = 0.0;
                    Eigen::Vector2d line_vec = p2 - p1;
                    double line_vec_norm_sq = line_vec.squaredNorm(); // 预计算分母

                    // 处理 p1 和 p2 几乎相同的退化情况
                    if (line_vec_norm_sq < 1e-9) {
                        RCLCPP_WARN(this->get_logger(), "线性度检查中发现退化线段 (p1~=p2)。");
                        max_deviation_sq = std::numeric_limits<double>::max(); // 强制线性度检查失败
                    } else {
                        // 计算其他点到直线 p1-p2 的最大平方偏差
                        for (size_t k = 0; k < closest_cluster.size(); ++k) {
                            if (k == idx1 || k == idx2) continue; // 跳过端点
                            const Eigen::Vector2d pk = closest_cluster[k].point;
                            Eigen::Vector2d p_vec = pk - p1;
                            // 使用叉乘计算距离: dist^2 = |line_vec x p_vec|^2 / |line_vec|^2
                            double cross_product_mag = std::abs(line_vec.x() * p_vec.y() - line_vec.y() * p_vec.x());
                            double deviation_sq = (cross_product_mag * cross_product_mag) / line_vec_norm_sq;
                            max_deviation_sq = std::max(max_deviation_sq, deviation_sq);
                        }
                    }

                    double max_deviation = std::sqrt(max_deviation_sq);
                    // 根据聚类阈值定义厚度阈值
                    double line_thickness_threshold = cluster_threshold_ * line_thickness_factor_;

                    RCLCPP_DEBUG(this->get_logger(), "聚类与直线的最大偏差: %.4f 米. 阈值: < %.4f",
                                max_deviation, line_thickness_threshold);

                    // 如果点足够靠近直线，则形状检查通过
                    if (max_deviation < line_thickness_threshold) {
                        shape_ok = true;
                        RCLCPP_DEBUG(this->get_logger(), "几何检查通过 (长度=%.3f, 最大偏差=%.4f)", max_dist, max_deviation);
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "几何检查失败: 线性度失败 (最大偏差=%.4f >= 阈值=%.4f)", max_deviation, line_thickness_threshold);
                    }
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "几何检查失败: 长度失败 (最大跨度=%.3f, 要求=[%.3f, %.3f])", max_dist, lower_bound, upper_bound);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "在聚类 (大小 %zu) 中未能找到有效的最远点 (max_dist_sq=%.1E)。", closest_cluster.size(), max_dist_sq);
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "聚类太小 (%zu 个点)，无法进行几何检查。", closest_cluster.size());
        }
        // --- 几何验证结束 ---


        // === 阶段 5: 角度计算 (如果形状OK则使用PCA以保证稳定性) 与姿态估计 ===
        if (shape_ok) {
            double rotation_angle = 0.0; // 初始化角度

            // 为通过验证的聚类计算协方差矩阵
            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for (const auto& p_info : closest_cluster) {
                Eigen::Vector2d p_centered = p_info.point - closest_centroid; // 以质心为中心
                cov += p_centered * p_centered.transpose();
            }
            // 如果聚类大小 > 1，使用 N-1 计算样本协方差
            if (closest_cluster.size() > 1) {
                 cov /= static_cast<double>(closest_cluster.size() - 1);
            }

            // 执行特征分解（PCA的核心）
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
            if (solver.info() == Eigen::Success) {
                // 获取最大特征值对应的特征向量（第一主成分）
                // Eigen 将特征值从小到大排序，所以索引 1 是最大的。
                Eigen::Vector2d principal_axis = solver.eigenvectors().col(1);
                // 使用 atan2 基于主轴计算角度，以保证稳定性
                rotation_angle = std::atan2(principal_axis.y(), principal_axis.x());

                RCLCPP_DEBUG(this->get_logger(), "使用PCA计算的角度: %.1f 度", rotation_angle * 180.0/M_PI);

                // === 可选：角度平滑 ===
                if (enable_angle_smoothing_) {
                    if (last_valid_angle_.has_value()) {
                         // 计算最短角度差，处理环绕
                         double angle_diff = angleDifference(rotation_angle, *last_valid_angle_);
                         // 应用指数移动平均 (EMA) 滤波
                         // new_angle = old_angle + alpha * difference, 其中 alpha = (1 - factor)
                         rotation_angle = *last_valid_angle_ + (1.0 - angle_smoothing_factor_) * angle_diff;
                         // 平滑后将角度归一化回 [-pi, pi]
                         rotation_angle = std::atan2(std::sin(rotation_angle), std::cos(rotation_angle));
                         RCLCPP_DEBUG(this->get_logger(), "平滑后的角度: %.1f 度", rotation_angle * 180.0/M_PI);
                    }
                    // 存储可能经过平滑的角度，供下一次迭代使用
                    last_valid_angle_ = rotation_angle;
                }

                // === 阶段 6: 发布结果 ===
                estimateCubePose(closest_centroid, rotation_angle, scan_msg->header);
                publishFilteredScan(closest_cluster, scan_msg); // 发布属于立方体的点

            } else {
                // 处理 PCA 失败
                RCLCPP_WARN(this->get_logger(), "角度计算期间特征分解失败。跳过本次姿态估计。");
                if (enable_angle_smoothing_) { last_valid_angle_ = std::nullopt; } // 失败时重置平滑状态
                publishFilteredScan({}, scan_msg); // 发布空扫描
            }
        } else {
             // 如果几何检查失败
             if (enable_angle_smoothing_) { last_valid_angle_ = std::nullopt; } // 重置平滑状态
             publishFilteredScan({}, scan_msg); // 发布空扫描
        }
    }

    // --- 发布相关的辅助函数 ---

    /**
     * @brief 创建并发布估计的姿态（PoseStamped 消息）。
     * @param centroid 计算出的检测到的立方体面的质心（位置）。
     * @param rotation_angle 计算出的方向（绕Z轴的角度，弧度）。
     * @param header 原始 LaserScan 消息的 header，用于时间戳和坐标系ID。
     */
    void estimateCubePose(const Eigen::Vector2d& centroid,
                          double rotation_angle,
                          const std_msgs::msg::Header& header) {
        auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose_msg->header = header; // 使用原始 header 保持一致性
        pose_msg->pose.position.x = centroid.x();
        pose_msg->pose.position.y = centroid.y();
        pose_msg->pose.position.z = 0.0; // 假设立方体在地面上 (z=0)

        // 将绕 Z 轴的旋转角转换为四元数
        pose_msg->pose.orientation.x = 0.0;
        pose_msg->pose.orientation.y = 0.0;
        pose_msg->pose.orientation.z = std::sin(rotation_angle / 2.0);
        pose_msg->pose.orientation.w = std::cos(rotation_angle / 2.0);

        pose_pub_->publish(std::move(pose_msg));

        // 同时发布可视化标记
        publishCubeMarker(centroid, rotation_angle, header);

        RCLCPP_INFO(this->get_logger(),
                   "检测到立方体位于 (%.2f, %.2f)，旋转角度 %.1f 度",
                   centroid.x(), centroid.y(), rotation_angle * 180.0 / M_PI);
    }

    /**
     * @brief 创建并发布用于 RViz 可视化的 CUBE 标记。
     * @param position 标记的位置（质心）。
     * @param rotation_angle 方向（绕Z轴的角度，弧度）。
     * @param header 原始 LaserScan 消息的 header。
     */
    void publishCubeMarker(const Eigen::Vector2d& position, double rotation_angle,
                         const std_msgs::msg::Header& header) {
        auto marker = std::make_unique<visualization_msgs::msg::Marker>();
        marker->header = header;
        marker->ns = "cube_detector"; // 标记的命名空间
        marker->id = 0;              // 此标记的唯一ID
        marker->type = visualization_msgs::msg::Marker::CUBE;
        marker->action = visualization_msgs::msg::Marker::ADD; // 添加/修改标记

        // 设置标记姿态
        marker->pose.position.x = position.x();
        marker->pose.position.y = position.y();
        marker->pose.position.z = 0.0; // 将标记底部置于 z=0

        marker->pose.orientation.x = 0.0;
        marker->pose.orientation.y = 0.0;
        marker->pose.orientation.z = std::sin(rotation_angle / 2.0);
        marker->pose.orientation.w = std::cos(rotation_angle / 2.0);

        // 设置标记尺寸（使用期望的立方体尺寸）
        marker->scale.x = cube_size_;
        marker->scale.y = cube_size_;
        marker->scale.z = cube_size_; // 给定高度以便可视化

        // 设置标记颜色（例如，绿色，半透明）
        marker->color.r = 0.0f;
        marker->color.g = 1.0f;
        marker->color.b = 0.0f;
        marker->color.a = 0.7f;

        // 设置标记生命周期（如果不更新，在 RViz 中持续显示的时间）
        marker->lifetime = rclcpp::Duration::from_seconds(0.5); // 例如，0.5秒

        marker_pub_->publish(std::move(marker));
    }

    /**
     * @brief 创建并发布一个新的 LaserScan 消息，其中只包含属于检测到的聚类（立方体）的点。
     *        其他距离值设置为无穷大。
     * @param cluster_points 属于检测到的聚类的 PointInfo 结构体向量。
     * @param original_scan 原始 LaserScan 消息的共享指针，用于获取元数据。
     */
    void publishFilteredScan(const std::vector<PointInfo>& cluster_points,
                             const sensor_msgs::msg::LaserScan::SharedPtr original_scan)
    {
        auto filtered_scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        // 复制原始扫描的元数据
        filtered_scan_msg->header = original_scan->header;
        filtered_scan_msg->angle_min = original_scan->angle_min;
        filtered_scan_msg->angle_max = original_scan->angle_max;
        filtered_scan_msg->angle_increment = original_scan->angle_increment;
        filtered_scan_msg->time_increment = original_scan->time_increment;
        filtered_scan_msg->scan_time = original_scan->scan_time;
        filtered_scan_msg->range_min = original_scan->range_min;
        filtered_scan_msg->range_max = original_scan->range_max;

        // 初始化 ranges 为无穷大（无效值）
        filtered_scan_msg->ranges.assign(original_scan->ranges.size(), std::numeric_limits<float>::infinity());
        // 如果存在 intensities，也进行初始化
        if (!original_scan->intensities.empty()) {
            filtered_scan_msg->intensities.assign(original_scan->intensities.size(), 0.0f);
        }

        // 填充属于聚类的点的原始 range/intensity 值
        for (const auto& p_info : cluster_points) {
            // 访问数组前检查索引有效性
            if (p_info.index < filtered_scan_msg->ranges.size()) {
                 // 复制对应索引处的原始距离值
                 filtered_scan_msg->ranges[p_info.index] = original_scan->ranges[p_info.index];
                 // 如果可用，复制强度值
                 if (!original_scan->intensities.empty() && p_info.index < filtered_scan_msg->intensities.size()) {
                     filtered_scan_msg->intensities[p_info.index] = original_scan->intensities[p_info.index];
                 }
            } else {
                // 如果 PointInfo 构造正确，这不应发生
                RCLCPP_WARN(this->get_logger(), "在聚类点中发现无效索引 %zu，超过扫描大小 %zu。",
                            p_info.index, filtered_scan_msg->ranges.size());
            }
        }

        filtered_scan_pub_->publish(std::move(filtered_scan_msg));
    }

}; // CubeDetector 类结束

// --- 主函数 ---
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // 可选：通过命令行参数或节点选项设置日志级别
    // rclcpp::NodeOptions options;
    // options.arguments({"--ros-args", "--log-level", "debug"}); // 示例：启用调试日志
    auto node = std::make_shared<CubeDetector>(); // 如果创建了选项，则传递: std::make_shared<CubeDetector>(options)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}