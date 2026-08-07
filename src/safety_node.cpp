/**
 * safety_node: 局部安全层 ROS2 节点
 *
 * 职责: 实例化 mechdog_navigation 纯算法库 (SensorFusion + PathPlanner),
 *       定时执行传感器融合, 输出速度指令到 /cmd_vel, 并发布融合结果。
 *
 * 接入师兄全局层 (Nav2):
 *   - 订阅 /scan       (激光雷达, 可选): 远距避障参考 (当前仅记录, 不参与融合)
 *   - 订阅 /cmd_vel_in (Nav2 输出, 可选): 作为速度仲裁输入 (预留)
 *   - 发布 /cmd_vel    (最终): 本节点可覆盖 Nav2 输出 (悬崖/近距障碍时优先)
 *
 * 数据源:
 *   - 默认模拟模式 (use_simulated=true), PC 可直接运行验证融合链路
 *   - 真机模式: 改 use_simulated=false, 并由 sensor 节点提供数据 (见 README)
 *
 * 构建: colcon build --packages-select mechdog_navigation_ros
 * 运行: ros2 run mechdog_navigation_ros safety_node
 */
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

// 纯算法库
#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_ir.h"
#include "sensor_fusion.h"
#include "path_planner.h"

using namespace mechdog;
using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {
        // 参数: use_simulated (默认 true, PC 模拟)
        use_simulated_ = this->declare_parameter("use_simulated", true);

        // ---- 初始化算法库 ----
        astra_ = std::make_unique<AstraProDriver>(use_simulated_);
        ultrasonic_ = std::make_unique<UltrasonicArrayDriver>(get_ultrasonic_layout());
        ir_ = std::make_unique<InfraRedSensor>(use_simulated_);
        fusion_ = std::make_unique<SensorFusion>(astra_.get(), ultrasonic_.get(), ir_.get());
        planner_ = std::make_unique<PathPlanner>();

        astra_->start();

        // ---- ROS2 接口 ----
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        fusion_pub_ = this->create_publisher<std_msgs::msg::String>("fusion_result", 10);

        // 订阅师兄雷达 (可选, 当前仅记录日志)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_ranges_ = msg->ranges;
            });

        // 定时器: 10Hz 融合周期
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SafetyNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(),
            "safety_node 启动: use_simulated=%s, 融合周期 10Hz",
            use_simulated_ ? "true" : "false");
    }

    ~SafetyNode() override {
        if (astra_) astra_->stop();
    }

private:
    void on_timer() {
        // 1. 融合 (纯算法, 无 ROS2 依赖)
        auto result = fusion_->fuse();

        // 2. 规划
        auto cmd = planner_->plan(result);

        // 3. 发布速度指令
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = cmd.linear;
        twist.angular.z = cmd.angular;
        cmd_vel_pub_->publish(twist);

        // 4. 发布融合结果 (JSON 字符串, 调试/巡检决策)
        auto msg = std_msgs::msg::String();
        msg.data = fusion_to_json(result, cmd);
        fusion_pub_->publish(msg);

        // 5. 日志 (10Hz 节流: 每 10 帧打一次)
        if (++tick_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "env=%d cliff=%s min_fwd=%.2fm action=%d vel=(%.2f, %.2f) scan=%zu",
                static_cast<int>(result.environment),
                result.cliff_detected ? "YES" : "no",
                result.min_forward_distance_m,
                static_cast<int>(result.recommended_action),
                cmd.linear, cmd.angular,
                scan_ranges_.size());
        }
    }

    // 算法库成员
    std::unique_ptr<AstraProDriver> astra_;
    std::unique_ptr<UltrasonicArrayDriver> ultrasonic_;
    std::unique_ptr<InfraRedSensor> ir_;
    std::unique_ptr<SensorFusion> fusion_;
    std::unique_ptr<PathPlanner> planner_;

    // ROS2 接口
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fusion_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool use_simulated_ = true;
    unsigned int tick_ = 0;
    std::vector<float> scan_ranges_;  // 雷达数据缓存 (预留)

    // 融合结果 -> JSON (供 /fusion_result 调试与巡检决策)
    static std::string fusion_to_json(const FusionResult& r, const VelocityCmd& v) {
        std::ostringstream oss;
        oss << "{\"timestamp\":" << r.timestamp
            << ",\"environment\":" << static_cast<int>(r.environment)
            << ",\"cliff\":" << (r.cliff_detected ? "true" : "false")
            << ",\"min_fwd_m\":" << r.min_forward_distance_m
            << ",\"action\":" << static_cast<int>(r.recommended_action)
            << ",\"astra_w\":" << r.effective_astra_weight
            << ",\"ultra_w\":" << r.effective_ultrasonic_weight
            << ",\"vx\":" << v.linear
            << ",\"wz\":" << v.angular << "}";
        return oss.str();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}
