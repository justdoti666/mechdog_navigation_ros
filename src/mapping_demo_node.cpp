/**
 * mapping_demo_node — 建图端到端演示节点 (P4 demo)
 *
 * 链路 (全真实管线, 仅深度源为合成):
 *   /odom_dry_run (师兄 base_cmd_vel_odom_node, cmd_vel 开环积分)
 *     → Pose2D (tf 帧合成的机器人位姿)
 *   合成深度图 (模拟一堵 3m 处的墙, 随位姿可见)
 *     → mechdog::depth_to_cloud        (P0 反投影, 真实代码)
 *     → mechdog::transform_to_base     (P0 外参变换, 真实代码)
 *     → mechdog::OccupancyGridMap::insert_cloud (P4 建图, 真实代码)
 *     → /map (nav_msgs/OccupancyGrid, 1Hz) + PGM 落盘 (~/mechdog_map.pgm)
 *
 * 用途: 在无真机/无相机条件下验证 "里程计位姿 → 点云 → 占据栅格"
 * 全链路, 为后续接 Astra 真深度流做准备。
 *
 * 用法:
 *   终端1: ros2 launch quadruped_base base_odom_dry_run.launch.py
 *   终端2: ros2 run mechdog_navigation_ros mapping_demo_node
 *   终端3: ros2 topic pub -r 10 /odom_test_cmd_vel geometry_msgs/msg/Twist \
 *            "{linear: {x: 0.1}}"
 *   查看: ros2 topic echo /map --once | head -20;  PGM: ~/mechdog_map.pgm
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "config.h"
#include "point_cloud.h"
#include "mapping.h"

using namespace std::chrono_literals;

namespace {

double yaw_from_quaternion(double z, double w) {
    // 偏航角提取 (单圈足够, odom demo 不做多圈跟踪)
    return 2.0 * std::atan2(z, w);
}

} // namespace

namespace mechdog_ros {

class MappingDemoNode : public rclcpp::Node {
public:
    MappingDemoNode() : Node("mapping_demo_node") {
        // --- 参数 ---
        declare_parameter<std::string>("odom_topic", "/odom_dry_run");
        declare_parameter<std::string>("map_topic", "/map");
        declare_parameter<std::string>("pgm_path", "/tmp/mechdog_map.pgm");
        declare_parameter<double>("frame_period_sec", 0.5);  // 2Hz 深度帧
        declare_parameter<int>("frame_width", 64);           // 合成分辨率(小图够用)
        declare_parameter<int>("frame_height", 48);
        declare_parameter<double>("wall_dist_m", 3.0);       // 模拟墙距离

        odom_topic_ = get_parameter("odom_topic").as_string();
        pgm_path_   = get_parameter("pgm_path").as_string();
        frame_w_    = get_parameter("frame_width").as_int();
        frame_h_    = get_parameter("frame_height").as_int();
        wall_dist_  = get_parameter("wall_dist_m").as_double();
        const double period =
            get_parameter("frame_period_sec").as_double();

        // --- 内参按帧分辨率同比缩放 (默认值是 640x480 的;
        //     小尺寸帧不缩放会导致像素挤在虚拟大图边缘, 光线方向全偏) ---
        K_.fx *= static_cast<double>(frame_w_) / 640.0;
        K_.fy *= static_cast<double>(frame_h_) / 480.0;
        K_.cx = frame_w_ / 2.0;
        K_.cy = frame_h_ / 2.0;

        // --- 订阅 odom ---
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                last_pose_.x = msg->pose.pose.position.x;
                last_pose_.y = msg->pose.pose.position.y;
                last_pose_.theta = yaw_from_quaternion(
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
                have_pose_ = true;
            });

        // --- 发布地图 ---
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            get_parameter("map_topic").as_string(), 1);

        // --- 定时: 合成深度帧 → 真实管线 → 建图 ---
        frame_timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            [this]() { on_frame(); });

        // --- 定时: 1Hz 发布地图 + 存 PGM ---
        map_timer_ = create_wall_timer(1s, [this]() { publish_map(); });

        RCLCPP_INFO(get_logger(),
            "mapping_demo_node started: odom=%s frame=%dx%d@%.1fHz wall=%.1fm "
            "pgm=%s",
            odom_topic_.c_str(), frame_w_, frame_h_, 1.0 / period,
            wall_dist_, pgm_path_.c_str());
    }

private:
    void on_frame() {
        if (!have_pose_) return;

        // --- 合成深度图: 相机正前方 wall_dist_ 处一面横墙 ---
        // (640x480 太大没必要; 64x48 已能覆盖 FOV 内墙)
        depth_.assign(static_cast<size_t>(frame_w_) * frame_h_, 0);
        for (int r = 0; r < frame_h_; ++r) {
            for (int c = 0; c < frame_w_; ++c) {
                // 边缘像素加角度衰减模拟真实墙的深度分布 (针孔模型):
                // 视场角外无墙 → 0 (无效), 视场角内 = wall_dist_/cos(角度)
                const double u = (c - K_.cx) / K_.fx;
                const double v = (r - K_.cy) / K_.fy;
                const double ray_len = std::sqrt(1.0 + u * u + v * v);
                const double d = wall_dist_ * ray_len; // 平面墙的深度值
                depth_[static_cast<size_t>(r) * frame_w_ + c] =
                    static_cast<uint16_t>(d * 1000.0);
            }
        }

        // --- 真实管线 P0: 反投影 + base 变换 ---
        mechdog::PointCloud cloud_optical, cloud_base;
        mechdog::depth_to_cloud(depth_.data(), frame_w_, frame_h_, K_,
                                cloud_optical);
        mechdog::transform_to_base(cloud_optical, E_, cloud_base);

        // --- 真实管线 P4: 累积建图 ---
        // 注: 合成帧是悬浮平面墙(无地面), 用 insert_cloud 原入口 —
        //     filtered 入口会做地面分割, 对合成分布行为不可预测;
        //     真机链路请参考算法库 tools/mapping_real_test (用 filtered)
        map_.insert_cloud(cloud_base, last_pose_);
        ++frames_;
    }

    void publish_map() {
        if (frames_ == 0) return;

        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.frame_id = "odom_dry_run";
        msg.header.stamp = now();
        msg.info.resolution = map_.resolution();
        msg.info.width = static_cast<uint32_t>(map_.width());
        msg.info.height = static_cast<uint32_t>(map_.height());
        // 原点 = 地图左下角世界坐标 (原点居中 → -5m,-5m)
        msg.info.origin.position.x = -5.0;
        msg.info.origin.position.y = -5.0;
        msg.data.resize(map_.width() * map_.height());
        for (int row = 0; row < map_.height(); ++row) {
            for (int col = 0; col < map_.width(); ++col) {
                const int s = map_.occ_state(col, row);
                msg.data[static_cast<size_t>(row) * map_.width() + col] =
                    static_cast<int8_t>(s); // -1/0/100 ROS 语义
            }
        }
        map_pub_->publish(msg);

        if (map_.save_pgm(pgm_path_)) {
            RCLCPP_INFO(get_logger(), "map: %s frames=%d %s",
                        pgm_path_.c_str(), frames_, map_.stats().c_str());
        } else {
            RCLCPP_WARN(get_logger(), "PGM save failed: %s", pgm_path_.c_str());
        }
    }

    // --- 状态 ---
    std::string odom_topic_, pgm_path_;
    int frame_w_ = 64, frame_h_ = 48;
    double wall_dist_ = 3.0;
    int frames_ = 0;
    bool have_pose_ = false;
    mechdog::Pose2D last_pose_{};
    std::vector<uint16_t> depth_;
    // 内参: 构造函数里按 frame_w_/frame_h_ 参数同比缩放 (见上)
    mechdog::CameraIntrinsics K_{};      // 默认值 = 640x480 基准
    mechdog::CameraExtrinsics E_{};      // 默认外参 (前 0.12m, 俯 15°)
    mechdog::OccupancyGridMap map_{mechdog::MapConfig{}, 0.25};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr frame_timer_, map_timer_;
};

} // namespace mechdog_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mechdog_ros::MappingDemoNode>());
    rclcpp::shutdown();
    return 0;
}
