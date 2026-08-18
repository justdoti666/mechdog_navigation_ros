/**
 * safety_node: 局部安全层 ROS2 节点
 *
 * 职责: 实例化 mechdog_navigation 纯算法库 (SensorFusion + PathPlanner),
 *       定时执行传感器融合, 输出速度指令, 并发布融合结果。
 *
 * 接入师兄全局层 (Nav2 + quadruped_ws 安全闸门):
 *   - 发布 /unsafe/cmd_vel (默认, 参数 cmd_vel_topic 可改):
 *       师兄 cmd_vel_safety_gate_node 订阅它, 经 estop/超时/限幅检查后
 *       转发到 /cmd_vel, 再经 wheel_board_bridge_node 发 STM32。
 *       (直接发 /cmd_vel 会绕过安全闸门, 违反安全分层)
 *   - 订阅 /scan       (激光雷达, sensor_data QoS, 可选): 远距避障参考 (当前仅记录)
 *   - 发布 /fusion_result (JSON, 调试/巡检决策)
 *
 * 数据源:
 *   - 默认模拟模式 (use_simulated=true), PC 可直接运行验证融合链路
 *   - 真机模式: 改 use_simulated=false, 并由 sensor 节点提供数据 (见 README)
 *
 * 构建: colcon build --packages-select mechdog_navigation_ros
 * 运行: ros2 run mechdog_navigation_ros safety_node
 *       # 若想直接接管 /cmd_vel (不经闸门, 仅测试): --ros-args -p cmd_vel_topic:=/cmd_vel
 */
#include <chrono>
#include <condition_variable>
#include <memory>
#include <sstream>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

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
        // 参数: cmd_vel_topic (默认 /unsafe/cmd_vel —— 师兄 quadruped_ws 的 cmd_vel_safety_gate_node
        //       订阅 /unsafe/cmd_vel 作为闸门输入, 经安全检查后转发到 /cmd_vel;
        //       直接发 /cmd_vel 会绕过师兄的安全闸门, 违反安全分层)
        cmd_vel_topic_ = this->declare_parameter("cmd_vel_topic", "/unsafe/cmd_vel");

        // ---- 初始化算法库 ----
        astra_ = std::make_unique<AstraProDriver>(use_simulated_);
        ultrasonic_ = std::make_unique<UltrasonicArrayDriver>(get_ultrasonic_layout());
        ir_ = std::make_unique<InfraRedSensor>(use_simulated_);
        fusion_ = std::make_unique<SensorFusion>(astra_.get(), ultrasonic_.get(), ir_.get());
        planner_ = std::make_unique<PathPlanner>();

        astra_->start();

        // H3: 融合移出 timer 线程 —— 真机 read_all 最坏 ~350-490ms (> 200ms 周期),
        // 同步 fuse() 会让 timer 回调漂移累积、发布流跌破 5Hz 并逼近上游闸门 0.5s
        // 超时 (机器人间歇零速)。独立融合线程持续 fuse() (周期 = read_all 耗时,
        // 真机自然 ~3Hz; 模拟 ~8Hz), timer 200ms 只发布最新缓存结果:
        // 发布流恒 5Hz, 闸门永不超时, 数据新鲜度最坏 ~350ms (步行速度可接受)。
        fusion_running_ = true;
        fusion_thread_ = std::thread([this]() {
            while (true) {
                auto result = fusion_->fuse();
                {
                    std::lock_guard<std::mutex> lock(result_mutex_);
                    latest_result_ = result;
                    have_result_ = true;
                    last_fusion_update_ = std::chrono::steady_clock::now();
                }
                if (!fusion_running_.load()) break;  // 剩余一次 fuse 后立刻响应停止
            }
            // H5: 通知退出 (供 stop_fusion_thread 带超时等待, 避免 join 挂死)
            {
                std::lock_guard<std::mutex> lk(fusion_exit_mutex_);
                fusion_exited_ = true;
            }
            fusion_exit_cv_.notify_all();
        });

        // ---- ROS2 接口 ----
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        fusion_pub_ = this->create_publisher<std_msgs::msg::String>("fusion_result", 10);

        // 订阅师兄雷达 (可选, 当前仅记录日志)
        // QoS: sensor_data —— 与师兄 lidar_obstacle_node 一致 (rplidar_ros 发布 /scan 用 sensor_data)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_ranges_ = msg->ranges;
            });

        // 定时器: 5Hz 发布最新融合结果 (融合本身在独立线程, 见上 H3 说明)
        timer_ = this->create_wall_timer(
            200ms, std::bind(&SafetyNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(),
            "safety_node 启动: use_simulated=%s, 融合周期 5Hz",
            use_simulated_ ? "true" : "false");
    }

    ~SafetyNode() override {
        // 先停融合线程 (join, 确保不再访问算法库), 再停 Astra 采集
        stop_fusion_thread();
        if (astra_) astra_->stop();
    }

private:
    // H5 修复: 若融合线程已卡死在阻塞读 (C1 场景), 无超时 join() 会永久挂起 ->
    // 析构/spin 退出挂死。用带超时的 wait_for 等待退出通知, 超时则 detach 兜底。
    // R2: 等待提到 2s —— 真机最坏 fuse() 周期 ~490ms, 正常退出远快于 2s; 2s 后仍
    // 未退说明线程真卡死在阻塞读, detach 概率近乎零。detach 的 UAF 窗口 (线程稍后从
    // 阻塞恢复会访问已析构 this) 因此也缩到仅剩"真卡死 + 进程已开始析构"的极端场景,
    // 由进程退出兜底 (OS 回收线程, 不再访问成员)。
    void stop_fusion_thread() {
        fusion_running_ = false;
        if (!fusion_thread_.joinable()) return;
        {
            std::unique_lock<std::mutex> lk(fusion_exit_mutex_);
            bool exited = fusion_exit_cv_.wait_for(
                lk, std::chrono::milliseconds(2000), [this] { return fusion_exited_; });
            if (!exited) {
                // 2s 内未退出 (融合线程真实卡死) -> detach 兜底, 不再等待
                fusion_thread_.detach();
                RCLCPP_WARN(this->get_logger(),
                    "融合线程 2s 内未退出, 已 detach 兜底 (疑似卡死在阻塞读)");
                return;
            }
        }
        if (fusion_thread_.joinable()) fusion_thread_.join();
    }

    void on_timer() {
        // 1. 取最新融合结果 (融合线程写, 本回调读, 锁保护)
        FusionResult result;
        bool fresh = false;
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            if (!have_result_) return;  // 首帧未就绪: 不发 (上游闸门 0.5s 超时兜底)

            // C1 修复: 新鲜度看门狗。FusionResult.timestamp 存在但从未被消费 ——
            // 融合线程一旦静默卡死 (真机 USB 断开 / fuse() 阻塞 / 读传感器挂起),
            // 旧实现会以 5Hz 无限期发布同一份陈旧指令 (fail-unsafe: 上游闸门只判
            // "0.5s 内收到消息" 的链路存活, 无法识别内容过期)。
            // R1: 阈值 800ms = 真机最坏 fuse() 周期 (~490ms) × ~1.6 裕量 —— 原 500ms
            // 与上界贴边, 单次调度抖动超 500ms 会触发假性零速 (安全侧但间歇停摆)。
            auto now = std::chrono::steady_clock::now();
            fresh = (now - last_fusion_update_) <= std::chrono::milliseconds(800);
            if (fresh) {
                result = latest_result_;
            } else {
                have_result_ = false;  // 数据过期, 丢弃缓存
            }
        }

        // 2. 规划 (陈旧时生成本地零速指令, 不再用 planner)
        VelocityCmd cmd;
        if (fresh) {
            cmd = planner_->plan(result);
        } else {
            cmd.linear = 0.0; cmd.angular = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "融合结果陈旧(>500ms), 发布安全零速 (疑似融合线程卡死)");
        }

        // 3. 发布速度指令 (double -> float 显式窄化, 消除隐式转换警告)
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = static_cast<float>(cmd.linear);
        twist.angular.z = static_cast<float>(cmd.angular);
        cmd_vel_pub_->publish(twist);

        // 4. 发布融合结果 (JSON 字符串, 调试/巡检决策)
        auto msg = std_msgs::msg::String();
        msg.data = fusion_to_json(result, cmd);
        fusion_pub_->publish(msg);

        // 5. 日志 (5Hz 节流: 每 10 帧打一次, 即每 2 秒)
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

    // H3: 独立融合线程 + 最新结果缓存 (timer 回调只读缓存, 锁保护)
    std::thread fusion_thread_;
    std::atomic<bool> fusion_running_{false};
    std::mutex result_mutex_;
    FusionResult latest_result_;
    bool have_result_ = false;
    // C1: 最新融合结果产出时刻 (融合线程写, on_timer 读, 同一把锁保护)
    std::chrono::steady_clock::time_point last_fusion_update_{};
    // H5: 融合线程退出信号 (避免无超时 join 挂死)
    std::mutex fusion_exit_mutex_;
    std::condition_variable fusion_exit_cv_;
    bool fusion_exited_ = false;

    // ROS2 接口
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fusion_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool use_simulated_ = true;
    std::string cmd_vel_topic_ = "/unsafe/cmd_vel";
    unsigned int tick_ = 0;
    // 注: scan_ranges_ 由 /scan 回调写、timer 回调读, 单线程 executor 下串行安全;
    //     若改多线程 executor 需加锁
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
