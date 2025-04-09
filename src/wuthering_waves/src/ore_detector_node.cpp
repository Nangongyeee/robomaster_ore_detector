#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

class CubeDetector : public rclcpp::Node {
public:
    CubeDetector() : Node("cube_detector") {
        // 参数设置
        this->declare_parameter("cube_size", 0.3);  // 正方体边长，单位米
        this->declare_parameter("fov_angle", 90.0); // FOV角度，单位度
        this->declare_parameter("cluster_threshold", 0.1); // 点云聚类阈值，单位米
        this->declare_parameter("min_points_per_cluster", 5); // 最小点数量判定为一个聚类
        
        cube_size_ = this->get_parameter("cube_size").as_double();
        fov_angle_ = this->get_parameter("fov_angle").as_double();
        cluster_threshold_ = this->get_parameter("cluster_threshold").as_double();
        min_points_per_cluster_ = this->get_parameter("min_points_per_cluster").as_int();
        
        // 创建订阅者
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CubeDetector::scanCallback, this, std::placeholders::_1));
            
        // 创建发布者
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cube_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cube_marker", 10);
        
        RCLCPP_INFO(this->get_logger(), "Cube detector initialized with cube size: %.2f m", cube_size_);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        // 仅处理FOV内的点云数据
        std::vector<Eigen::Vector2d> points;
        double half_fov = fov_angle_ / 2.0 * M_PI / 180.0; // 转换为弧度
        
        // 提取FOV内的有效点
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            if (std::abs(angle) > half_fov) continue; // 跳过FOV外的点
            
            double range = scan_msg->ranges[i];
            if (!std::isfinite(range) || range < scan_msg->range_min || range > scan_msg->range_max) {
                continue; // 跳过无效点
            }
            
            // 转换为笛卡尔坐标
            double x = range * cos(angle);
            double y = range * sin(angle);
            points.push_back(Eigen::Vector2d(x, y));
        }
        
        // 如果点数太少，直接返回
        if (points.size() < min_points_per_cluster_) {
            RCLCPP_DEBUG(this->get_logger(), "Not enough valid points in FOV");
            return;
        }
        
        // 简单聚类分割 - 基于欧几里得距离
        std::vector<std::vector<Eigen::Vector2d>> clusters;
        std::vector<bool> processed(points.size(), false);
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (processed[i]) continue;
            
            std::vector<Eigen::Vector2d> current_cluster;
            expandCluster(points, processed, i, current_cluster);
            
            if (current_cluster.size() >= min_points_per_cluster_) {
                clusters.push_back(current_cluster);
            }
        }
        
        // 找到最可能是正方体的聚类
        if (clusters.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No valid clusters found");
            return;
        }
        
        // 选择最佳候选聚类（这里简化为选择大小最接近预期的聚类）
        int best_cluster_idx = -1;
        double best_cluster_score = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            // 计算聚类的包围盒
            double min_x = std::numeric_limits<double>::max();
            double max_x = -std::numeric_limits<double>::max();
            double min_y = std::numeric_limits<double>::max();
            double max_y = -std::numeric_limits<double>::max();
            
            for (const auto& point : clusters[i]) {
                min_x = std::min(min_x, point.x());
                max_x = std::max(max_x, point.x());
                min_y = std::min(min_y, point.y());
                max_y = std::max(max_y, point.y());
            }
            
            double width = max_x - min_x;
            double height = max_y - min_y;
            
            // 计算聚类的偏离预期尺寸的程度
            double expected_perimeter = 4 * cube_size_;
            double actual_perimeter = 2 * (width + height);
            double score = std::abs(expected_perimeter - actual_perimeter);
            
            if (score < best_cluster_score) {
                best_cluster_score = score;
                best_cluster_idx = i;
            }
        }
        
        if (best_cluster_idx == -1) {
            RCLCPP_DEBUG(this->get_logger(), "No suitable cube candidate found");
            return;
        }
        
        // 对最佳聚类进行正方体位姿估计
        estimateCubePose(clusters[best_cluster_idx], scan_msg->header);
    }
    
    void expandCluster(const std::vector<Eigen::Vector2d>& points, 
                       std::vector<bool>& processed, 
                       size_t point_idx, 
                       std::vector<Eigen::Vector2d>& cluster) {
        processed[point_idx] = true;
        cluster.push_back(points[point_idx]);
        
        std::vector<size_t> neighbors;
        findNeighbors(points, point_idx, neighbors);
        
        for (size_t neighbor_idx : neighbors) {
            if (!processed[neighbor_idx]) {
                processed[neighbor_idx] = true;
                
                std::vector<size_t> neighbor_neighbors;
                findNeighbors(points, neighbor_idx, neighbor_neighbors);
                
                if (neighbor_neighbors.size() >= min_points_per_cluster_) {
                    for (size_t nn_idx : neighbor_neighbors) {
                        if (std::find(neighbors.begin(), neighbors.end(), nn_idx) == neighbors.end()) {
                            neighbors.push_back(nn_idx);
                        }
                    }
                }
                
                cluster.push_back(points[neighbor_idx]);
            }
        }
    }
    
    void findNeighbors(const std::vector<Eigen::Vector2d>& points, 
                       size_t point_idx, 
                       std::vector<size_t>& neighbors) {
        neighbors.clear();
        for (size_t i = 0; i < points.size(); ++i) {
            if (i == point_idx) continue;
            
            double dist = (points[i] - points[point_idx]).norm();
            if (dist <= cluster_threshold_) {
                neighbors.push_back(i);
            }
        }
    }
    
    void estimateCubePose(const std::vector<Eigen::Vector2d>& cluster, 
                          const std_msgs::msg::Header& header) {
        // 使用主成分分析估计正方体的方向
        Eigen::Vector2d centroid(0.0, 0.0);
        for (const auto& point : cluster) {
            centroid += point;
        }
        centroid /= cluster.size();
        
        // 构建协方差矩阵
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (const auto& point : cluster) {
            Eigen::Vector2d p_centered = point - centroid;
            cov += p_centered * p_centered.transpose();
        }
        cov /= cluster.size();
        
        // 计算特征向量和特征值
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
        Eigen::Vector2d principal_direction = solver.eigenvectors().col(1);
        
        // 计算旋转角度（相对于x轴）
        double rotation_angle = std::atan2(principal_direction.y(), principal_direction.x());
        
        // 创建并发布PoseStamped消息
        auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose_msg->header = header;
        pose_msg->pose.position.x = centroid.x();
        pose_msg->pose.position.y = centroid.y();
        pose_msg->pose.position.z = 0.0;
        
        // 四元数表示绕z轴的旋转
        pose_msg->pose.orientation.x = 0.0;
        pose_msg->pose.orientation.y = 0.0;
        pose_msg->pose.orientation.z = std::sin(rotation_angle / 2.0);
        pose_msg->pose.orientation.w = std::cos(rotation_angle / 2.0);
        
        pose_pub_->publish(std::move(pose_msg));
        
        // 创建并发布可视化标记
        publishCubeMarker(centroid, rotation_angle, header);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Cube detected at (%.2f, %.2f) with rotation %.2f degrees",
                   centroid.x(), centroid.y(), rotation_angle * 180.0 / M_PI);
    }
    
    void publishCubeMarker(const Eigen::Vector2d& position, double rotation_angle,
                         const std_msgs::msg::Header& header) {
        auto marker = std::make_unique<visualization_msgs::msg::Marker>();
        marker->header = header;
        marker->ns = "cube";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::CUBE;
        marker->action = visualization_msgs::msg::Marker::ADD;
        
        marker->pose.position.x = position.x();
        marker->pose.position.y = position.y();
        marker->pose.position.z = cube_size_ / 2.0;  // 将z设为正方体高度的一半
        
        marker->pose.orientation.x = 0.0;
        marker->pose.orientation.y = 0.0;
        marker->pose.orientation.z = std::sin(rotation_angle / 2.0);
        marker->pose.orientation.w = std::cos(rotation_angle / 2.0);
        
        marker->scale.x = cube_size_;
        marker->scale.y = cube_size_;
        marker->scale.z = cube_size_;
        
        marker->color.r = 0.0;
        marker->color.g = 1.0;
        marker->color.b = 0.0;
        marker->color.a = 0.7;
        
        marker->lifetime = rclcpp::Duration::from_seconds(0.5);  // 显示0.5秒
        
        marker_pub_->publish(std::move(marker));
    }

    // 成员变量
    double cube_size_;
    double fov_angle_;
    double cluster_threshold_;
    int min_points_per_cluster_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubeDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

