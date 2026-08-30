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
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"  // ROS-5: main 用 SingleThreadedExecutor (显式 include, 不依赖聚合头传递)
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

// 纯算法库
#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_ir.h"
#include "sensor_fusion.h"
#include "path_planner.h"
#include "point_cloud.h"

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

        // ---- 近场点云 (P3 起步): 深度帧反投影 -> camera_link 系 PointCloud2 ----
        // 定位: 近场局部 3D 感知, 喂 Nav2 voxel_layer 做悬空/立体障碍标记; 全局建图归师兄激光雷达.
        // 默认关 (行为不变); 真机注意本节点已独占相机, 勿再开第二个 Astra 进程 (serial 为空, 深度全失效).
        enable_pointcloud_ = this->declare_parameter("enable_pointcloud", false);
        cloud_topic_ = this->declare_parameter("cloud_topic", "/mechdog/point_cloud");
        cloud_frame_ = this->declare_parameter("cloud_frame", "camera_link");
        cloud_step_ = std::max(1, static_cast<int>(
            this->declare_parameter("cloud_downsample_step", 8)));

        // ---- RGB 回传 (替代支架相机/USB 相机): Astra RGB -> sensor_msgs/Image ----
        // 师兄的温度-视觉验证与 Foxglove 回传直接换图像源即可; Foxglove bridge 自带压缩.
        // 注意: Astra RGB 仅真机模式有数据 (模拟模式 get_color_frame 返回无效), 但参数照常生效.
        enable_rgb_ = this->declare_parameter("enable_rgb", false);
        rgb_topic_ = this->declare_parameter("rgb_topic", "/mechdog/rgb/image_raw");
        rgb_frame_ = this->declare_parameter("rgb_frame", "camera_link");
        rgb_fps_ = std::max(1, static_cast<int>(
            this->declare_parameter("rgb_fps", 10)));

        // ---- 初始化算法库 ----
        astra_ = std::make_unique<AstraProDriver>(use_simulated_);
        ultrasonic_ = std::make_unique<UltrasonicArrayDriver>(get_ultrasonic_layout());
        ir_ = std::make_unique<InfraRedSensor>(use_simulated_);
        fusion_ = std::make_unique<SensorFusion>(astra_.get(), ultrasonic_.get(), ir_.get());
        planner_ = std::make_unique<PathPlanner>();

        astra_->start();

        // H3: 融合移出 timer 线程。真机 fuse() = read_all(3 颗前向) + 微秒级融合计算,
        //   最坏 ~135ms (无回波或 echo 卡高, 均每颗 25ms×3 + 2×30ms 间隔; ALG-2 v2.3 校准,
        //   measure_distance 两段忙等单次只超时其一, 非 50ms/颗; 原 350-490ms 偏高)。
        //   虽 135ms < 200ms timer 周期, 仍独立线程: 发布恒 5Hz 不受 fuse 抖动影响,
        //   且新鲜度看门狗 (800ms, ROS-4) 兜底 fuse 阻塞 (USB 断开等极端情形)。
        //   模拟 ~60ms/16.7Hz, 真机典型 ~80-115ms/9-12Hz, 最坏 ~135ms/7.4Hz;
        //   timer 200ms 只发布最新缓存, 新鲜度最坏 ~135ms (步行可接受)。
        // ROS-2 (v2.2): 加 ≥100ms 最小周期门控, 防御性卫生 (避免后续简化传感器读后空转)。
        // ROS-3 (v2.2): stop 检查置于写共享状态之前 —— 析构中 fuse 返回则不再触碰 latest_result_,
        //   缩 detach UAF 窗口至"仅 fuse 阻塞中"的极端情形 (真机 USB 断开; 真正根治需可取消 I/O, 见 FIX_PLAN F10)。
        fusion_running_ = true;
        fusion_thread_ = std::thread([this]() {
            constexpr auto kMinCycle = std::chrono::milliseconds(100);  // ROS-2
            while (true) {
                auto t0 = std::chrono::steady_clock::now();
                auto result = fusion_->fuse();
                // ROS-3: 先判停止, 再决定是否写共享状态 (析构中不再访问成员)
                if (!fusion_running_.load()) break;
                {
                    std::lock_guard<std::mutex> lock(result_mutex_);
                    latest_result_ = result;
                    have_result_ = true;
                    last_fusion_update_ = std::chrono::steady_clock::now();
                }
                // 近场点云: 跟随融合节拍发布 (融合线程独占相机读取; rclcpp publish 线程安全)
                if (cloud_pub_) {
                    publish_pointcloud();
                }
                // RGB 回传: 节流到目标帧率后发布 Astra 彩色帧 (同一驱动实例, 免抢相机)
                if (rgb_pub_) {
                    publish_rgb_if_due();
                }
                // ROS-2: 速率门控 (fuse 自身已含 read_all sleep, 但防御性兜底)
                auto elapsed = std::chrono::steady_clock::now() - t0;
                if (elapsed < kMinCycle) {
                    std::this_thread::sleep_for(kMinCycle - elapsed);
                }
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
        if (enable_pointcloud_) {
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 5);
            RCLCPP_INFO(this->get_logger(),
                "近场点云已启用: topic=%s frame=%s 下采样步长=%d (配合 static TF %s -> base_link)",
                cloud_topic_.c_str(), cloud_frame_.c_str(), cloud_step_,
                cloud_frame_.c_str());
        }
        if (enable_rgb_) {
            rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(rgb_topic_, 5);
            RCLCPP_INFO(this->get_logger(),
                "RGB 回传已启用: topic=%s frame=%s 目标帧率=%dfps (真机模式出图, 模拟模式无彩色帧)",
                rgb_topic_.c_str(), rgb_frame_.c_str(), rgb_fps_);
        }

        // 订阅师兄雷达 (可选, 当前仅记录日志)
        // QoS: sensor_data —— 与师兄 lidar_obstacle_node 一致 (rplidar_ros 发布 /scan 用 sensor_data)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                // ROS-5 (v2.2): 加锁保护 (原仅靠单线程 executor 隐式串行; 切多线程 executor 会竞争)
                std::lock_guard<std::mutex> lk(scan_mutex_);
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
    // R2: 等待提到 2s —— 真机最坏 fuse() 周期 ~135ms (ALG-2 v2.3 校准, 原 ~490ms 偏高),
    //   正常退出远快于 2s; 2s 后仍
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

    // 近场点云 (P3 起步): 深度帧反投影 -> camera_link 系降采样 PointCloud2.
    // 由融合线程调用 (独占相机读取, 无跨线程共享, 无需加锁); rclcpp publish 线程安全.
    // stamp 用发布时刻 (静态 TF 对任意时刻有效, Nav2 voxel_layer 按"最新观测"消费).
    void publish_pointcloud() {
        AstraFrame frame = astra_->get_latest_frame();
        if (!frame.valid || frame.depth_map.empty() ||
            frame.depth_width <= 0 || frame.depth_height <= 0) {
            return;  // 首帧未就绪 / 真机帧失效 (H1 同口径)
        }
        PointCloud cloud_opt, cloud_link;
        depth_to_cloud(frame.depth_map.data(), frame.depth_width,
                       frame.depth_height, cloud_K_, cloud_opt);
        transform_optical_to_link(cloud_opt, cloud_link);

        // 降采样 + 序列化: x/y/z float32 + 4 字节 padding, point_step=16
        const size_t total = cloud_link.points.size();
        const size_t step = static_cast<size_t>(cloud_step_);
        std::vector<uint8_t> buf;
        buf.reserve((total / step + 1) * 16);
        for (size_t i = 0; i < total; i += step) {
            const auto& p = cloud_link.points[i];
            const float xyz[4] = {static_cast<float>(p.x), static_cast<float>(p.y),
                                  static_cast<float>(p.z), 0.0f};
            const auto* bytes = reinterpret_cast<const uint8_t*>(xyz);
            buf.insert(buf.end(), bytes, bytes + sizeof(xyz));
        }
        if (buf.empty()) return;

        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = cloud_frame_;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(buf.size() / 16);
        msg.is_dense = true;       // depth_to_cloud 已按 [0.6, 8.0]m 过滤无效像素
        msg.is_bigendian = false;
        msg.point_step = 16;
        msg.row_step = msg.point_step * msg.width;
        sensor_msgs::msg::PointField field;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        field.name = "x"; field.offset = 0; msg.fields.push_back(field);
        field.name = "y"; field.offset = 4; msg.fields.push_back(field);
        field.name = "z"; field.offset = 8; msg.fields.push_back(field);
        msg.data = std::move(buf);
        cloud_pub_->publish(msg);
    }

    // RGB 回传: Astra 彩色帧缓存 -> sensor_msgs/Image (rgb8). 按 rgb_fps 节流;
    // 真机模式 get_color_frame 返回窗口线程同款彩色缓存, 模拟模式无彩色数据 (跳过).
    void publish_rgb_if_due() {
        auto now = std::chrono::steady_clock::now();
        const auto min_interval =
            std::chrono::duration<double>(1.0 / static_cast<double>(rgb_fps_));
        if (last_rgb_pub_.time_since_epoch().count() != 0 &&
            now - last_rgb_pub_ < min_interval) {
            return;
        }
        ColorFrameData cf = astra_->get_color_frame();
        if (!cf.valid || cf.rgb.empty() || cf.width <= 0 || cf.height <= 0) {
            return;  // 模拟模式 / 彩色流未就绪
        }
        sensor_msgs::msg::Image msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = rgb_frame_;
        msg.height = static_cast<uint32_t>(cf.height);
        msg.width = static_cast<uint32_t>(cf.width);
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = static_cast<uint32_t>(cf.width) * 3;
        msg.data = std::move(cf.rgb);
        rgb_pub_->publish(std::move(msg));
        last_rgb_pub_ = now;
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
            // R1: 阈值 800ms ≈ 真机最坏 fuse() 周期 (~135ms, ALG-2 v2.3 校准) × 5.9 裕量 —— 原 500ms
            //   与旧上界 (~490ms) 贴边, 单次调度抖动超 500ms 会触发假性零速 (安全侧但间歇停摆)。
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
                "融合结果陈旧(>800ms), 发布安全零速 (疑似融合线程卡死)");  // ROS-4 v2.2: 文案与 800ms 阈值对齐
        }

        // 3. 发布速度指令 (geometry_msgs Twist 字段为 float64, 直接赋值即可;
        //    原先的 static_cast<float> 反而引入无谓的 float32 精度损失)
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = cmd.linear;
        twist.angular.z = cmd.angular;
        cmd_vel_pub_->publish(twist);

        // 4. 发布融合结果 (JSON 字符串, 调试/巡检决策)
        //    仅 fresh 时发布: 陈旧时 result 是默认空值 (action=FORWARD/min_fwd=8.0), 与零速 cmd 矛盾,
        //    会误导 /fusion_result 消费者。陈旧由 RCLCPP_WARN_THROTTLE 日志 + 零速 cmd 表达。
        if (fresh) {
            auto msg = std_msgs::msg::String();
            msg.data = fusion_to_json(result, cmd);
            fusion_pub_->publish(msg);
        }

        // 5. 日志 (5Hz 节流: 每 10 帧打一次, 即每 2 秒)
        if (++tick_ % 10 == 0) {
            // ROS-5: 加锁读 scan_ranges_ (与 /scan 回调同锁, 不再隐式依赖单线程 executor)
            size_t scan_n;
            { std::lock_guard<std::mutex> lk(scan_mutex_); scan_n = scan_ranges_.size(); }
            RCLCPP_INFO(this->get_logger(),
                "env=%s cliff=%s min_fwd=%.2fm action=%s vel=(%.2f, %.2f) scan=%zu",
                env_to_str(result.environment),
                result.cliff_detected ? "YES" : "no",
                result.min_forward_distance_m,
                action_to_str(result.recommended_action),  // ROS-6 v2.2: 枚举改字符串名
                cmd.linear, cmd.angular,
                scan_n);
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool use_simulated_ = true;
    std::string cmd_vel_topic_ = "/unsafe/cmd_vel";
    unsigned int tick_ = 0;
    // ROS-5 (v2.2): scan_ranges_ 加显式锁 (原仅靠单线程 executor 隐式串行, 现显式保护)
    std::mutex scan_mutex_;
    std::vector<float> scan_ranges_;  // 雷达数据缓存 (预留; /scan 回调写, on_timer 读)

    // 近场点云 (P3): 参数 + 内参 (FOV 反推, 真机标定后改 SDK 直读, 见设计文档 §3.2)
    bool enable_pointcloud_ = false;
    std::string cloud_topic_ = "/mechdog/point_cloud";
    std::string cloud_frame_ = "camera_link";
    int cloud_step_ = 8;
    CameraIntrinsics cloud_K_;

    // RGB 回传 (替代支架相机): 参数 + 发布节流状态 (仅融合线程访问, 无需锁)
    bool enable_rgb_ = false;
    std::string rgb_topic_ = "/mechdog/rgb/image_raw";
    std::string rgb_frame_ = "camera_link";
    int rgb_fps_ = 10;
    std::chrono::steady_clock::time_point last_rgb_pub_{};

    // ROS-6 (v2.2): 枚举改字符串名 (原 JSON 内嵌 int, 消费者需对照源码枚举值, 易错)
    static const char* env_to_str(EnvironmentType e) {
        switch (e) {
            case EnvironmentType::INDOOR:      return "INDOOR";
            case EnvironmentType::SEMI_INDOOR:return "SEMI_INDOOR";
            case EnvironmentType::OUTDOOR:    return "OUTDOOR";
            default:                          return "UNKNOWN";
        }
    }
    static const char* action_to_str(NavigationAction a) {
        switch (a) {
            case NavigationAction::STOP:        return "STOP";
            case NavigationAction::BACKWARD:     return "BACKWARD";
            case NavigationAction::TURN_LEFT:    return "TURN_LEFT";
            case NavigationAction::TURN_RIGHT:   return "TURN_RIGHT";
            case NavigationAction::SLOW_FORWARD: return "SLOW_FORWARD";
            case NavigationAction::FORWARD:      return "FORWARD";
            case NavigationAction::REACHED_GOAL:  return "REACHED_GOAL";
            default:                             return "UNKNOWN";
        }
    }

    // 融合结果 -> JSON (供 /fusion_result 调试与巡检决策)
    static std::string fusion_to_json(const FusionResult& r, const VelocityCmd& v) {
        std::ostringstream oss;
        // P3: 默认 6 位有效数字会把 epoch 秒 (~1.79e9) 截到小时级分辨率 (实测 1.78793e+09);
        //     统一 fixed(3): 时间戳毫秒级, 其余数值字段三位小数 (m/s / rad/s 分辨率足够)
        oss << std::fixed << std::setprecision(3)
            << "{\"timestamp\":" << r.timestamp
            << ",\"environment\":\"" << env_to_str(r.environment) << "\""
            << ",\"cliff\":" << (r.cliff_detected ? "true" : "false")
            << ",\"min_fwd_m\":" << r.min_forward_distance_m
            << ",\"action\":\"" << action_to_str(r.recommended_action) << "\""
            << ",\"astra_w\":" << r.effective_astra_weight
            << ",\"ultra_w\":" << r.effective_ultrasonic_weight
            << ",\"vx\":" << v.linear
            << ",\"wz\":" << v.angular << "}";
        return oss.str();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // ROS-5 (v2.2): 显式单线程 executor (scan_ranges_ 已加锁, 双重保险; 文档化不依赖隐式串行)
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<SafetyNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
