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
#include <cstring>
#include <cmath>
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
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"

// 方案A: 订阅独立 ultrasonic_node 发布的 /ultrasonic (mechdog_ultrasonic 包的消息)
#include "mechdog_ultrasonic/msg/ultrasonic_array.hpp"

// 纯算法库
#include "sensor_astra.h"
#include "sensor_ultrasonic.h"
#include "sensor_ir.h"
#include "sensor_fusion.h"
#include "path_planner.h"
#include "point_cloud.h"
#include "ground_segmentation.h"
#include "heightmap_2d5.h"     // 路1: 近场地形避障 (P1.5 2.5D 禁行格 → 融合决策)
#include "sensor_geometry.hpp"     // v2.6: 相机几何 → 地面高度先验
#include "safety_warmup.hpp"   // R4 (REVIEW): 启动预热 —— 等 bottom 首帧再首轮 fuse
#include "ultrasonic_source.hpp"   // v2.5: 超声来源解析 (真机拒绝模拟随机数)

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
        // R4 (REVIEW): 启动预热时长 (ms)。融合线程启动时先等底部线程产出首帧再首轮 fuse(),
        //   消除启动期 is_fall_risk() 因 !bottom_have_ 造成的一帧误急停 (~150ms)。
        //   默认 250ms: 就绪即返 (sim ~50-150ms), 有界不阻塞; 超时仍继续 (fail-closed 兜底)。
        warmup_ms_ = this->declare_parameter("warmup_ms", 250);

        // ---- 近场点云 (P3 起步): 深度帧反投影 -> camera_link 系 PointCloud2 ----
        // 定位: 近场局部 3D 感知, 喂 Nav2 voxel_layer 做悬空/立体障碍标记; 全局建图归师兄激光雷达.
        // 默认关 (行为不变); 真机注意本节点已独占相机, 勿再开第二个 Astra 进程 (serial 为空, 深度全失效).
        enable_pointcloud_ = this->declare_parameter("enable_pointcloud", false);
        cloud_topic_ = this->declare_parameter("cloud_topic", "/mechdog/point_cloud");
        cloud_frame_ = this->declare_parameter("cloud_frame", "camera_link");
        cloud_step_ = std::max(1, static_cast<int>(
            this->declare_parameter("cloud_downsample_step", 8)));
        // 负障碍话题 (P1): base_link 系坑/下行台阶标记点, 跟随 enable_pointcloud 开关
        neg_topic_ = this->declare_parameter("negative_topic", "/mechdog/negative_obstacles");

        // ---- RGB 回传 (替代支架相机/USB 相机): Astra RGB -> sensor_msgs/Image ----
        // 师兄的温度-视觉验证与 Foxglove 回传直接换图像源即可; Foxglove bridge 自带压缩.
        // 注意: Astra RGB 仅真机模式有数据 (模拟模式 get_color_frame 返回无效), 但参数照常生效.
        enable_rgb_ = this->declare_parameter("enable_rgb", false);
        rgb_topic_ = this->declare_parameter("rgb_topic", "/mechdog/rgb/image_raw");
        rgb_frame_ = this->declare_parameter("rgb_frame", "camera_link");
        rgb_fps_ = std::max(1, static_cast<int>(
            this->declare_parameter("rgb_fps", 10)));

        // ---- 超声来源 (v2.5): 真机上绝不允许"静默的模拟随机数"进安全链 ----
        // 背景: Pi 上没编 USE_WIRINGPI → 超声驱动是编译期模拟器, 数据带 valid=true,
        //   且模拟器"底部 5% 概率造悬崖" → 实测 5.7% 帧假 STOP。
        // auto(默认): use_simulated→simulated; /ultrasonic 有发布者→topic;
        //             GPIO 就绪→hardware; 都不满足→none (警告, 仅深度工作)
        // 其余取值: none | topic | hardware | simulated(仅台架)
        ultrasonic_requested_ = this->declare_parameter("ultrasonic_source", std::string("auto"));
        allow_simulated_ultrasonic_ =
            this->declare_parameter("allow_simulated_ultrasonic", false);
        ultrasonic_timeout_ms_ = std::max(50, static_cast<int>(
            this->declare_parameter("ultrasonic_timeout_ms", 500)));

        // ---- 相机几何 (v2.6): 台架 0.6m / 装机 0.18~0.2m —— **一处配置, 不再写死** ----
        // 动因 (真机实测 2026-09-13): ground_prior_z(-0.18) 是装机值 (假设相机离地 18cm);
        //   台架把相机架在 0.6m 时真地面落在先验窗之外 → 拟合器锁到别的面 →
        //   2.5D known/traversable 全部失去意义 (同帧实测: 先验 -0.18 → 可通行 0/35;
        //   先验 -0.6 → 可通行 22/35, 真地面在相机下方 0.657m)。
        // camera_height_m: 相机镜头离**地面**的高度; >0 时推导 ground_prior_z =
        //   -(camera_height_m - cloud_z) (见 src/sensor_geometry.hpp); 默认 -1 = 沿用装机默认
        // ground_prior_z / ground_prior_window: 直接指定 (优先于推导)
        // cloud_x/cloud_z/cloud_pitch_rad: 算法侧外参 —— 与 launch 的静态 TF **同源**,
        //   顺手修掉"TF 用 launch 值、算法用硬编码默认值"的不一致
        cloud_x_         = this->declare_parameter("cloud_x", 0.12);
        cloud_z_         = this->declare_parameter("cloud_z", 0.18);
        cloud_pitch_rad_ = this->declare_parameter("cloud_pitch_rad", 0.2617994);
        camera_height_m_ = this->declare_parameter("camera_height_m", -1.0);
        prior_window_m_  = this->declare_parameter("ground_prior_window", -1.0);
        const double prior_override = this->declare_parameter("ground_prior_z", -999.0);

        cloud_E_.x     = cloud_x_;
        cloud_E_.y     = 0.0;
        cloud_E_.z     = cloud_z_;
        cloud_E_.roll  = 0.0;
        cloud_E_.pitch = cloud_pitch_rad_;
        cloud_E_.yaw   = 0.0;
        if (prior_override > -900.0) {
            gseg_params_.ground_prior_z = prior_override;
        } else if (camera_height_m_ > 0.0) {
            gseg_params_.ground_prior_z =
                mechdog_ros::derived_ground_prior_z(camera_height_m_, cloud_z_);
        }
        if (prior_window_m_ > 0.0) gseg_params_.prior_window = prior_window_m_;
        RCLCPP_INFO(this->get_logger(),
            "相机几何: 离地 %.2fm%s 外参 x=%.2f z=%.2f pitch=%.4frad → 地面高度先验 z=%.3f (±%.2f) "
            "[台架: camera_height_m:=0.6 cloud_z:=0 | 装机: 改回 0.2]",
            camera_height_m_, camera_height_m_ > 0.0 ? "" : "(未设, 用仓库默认)",
            cloud_E_.x, cloud_E_.z, cloud_E_.pitch,
            gseg_params_.ground_prior_z, gseg_params_.prior_window);

        // ---- 深度来源 (v2.4): 无 Astra SDK 的机器 (如 Pi 5B) 用 ROS 话题喂深度 ----
        // 背景: safety_node 原本只能用 AstraProDriver 直读 SDK (真机) 或模拟帧;
        //   Pi 上没有 Orbbec Astra SDK, 但已有 ros2_astra_camera 在发深度话题 →
        //   本参数让节点订阅话题并把帧注入驱动, 下游 (融合/点云/2.5D/路1) 完全不变。
        //   "auto"      : use_simulated=true → simulated; 否则 编译了 SDK → sdk / 未编译 → topic
        //   "topic"     : 订阅 depth_topic (16UC1/mono16/32FC1) → inject_depth_frame
        //                 (不启动 SDK 采集线程 —— 否则注入帧会被采集线程覆盖)
        //   "sdk"       : Astra SDK 直读 (需 USE_ASTRA_SDK 编译)
        //   "simulated" : 模拟帧
        depth_source_ = this->declare_parameter("depth_source", std::string("auto"));
        depth_topic_ = this->declare_parameter("depth_topic",
            std::string("/camera/depth/image_raw"));
        depth_info_topic_ = this->declare_parameter("depth_info_topic",
            std::string("/camera/depth/camera_info"));
        // 话题超时 (ms): 超过则把深度帧标记为失效 (fail-closed, 只留超声) —— 不让旧帧继续参与决策
        depth_timeout_ms_ = std::max(50, static_cast<int>(
            this->declare_parameter("depth_timeout_ms", 500)));

        // ---- 初始化算法库 ----
        astra_ = std::make_unique<AstraProDriver>(use_simulated_);
        ultrasonic_ = std::make_unique<UltrasonicArrayDriver>(get_ultrasonic_layout());
        ir_ = std::make_unique<InfraRedSensor>(use_simulated_);
        fusion_ = std::make_unique<SensorFusion>(astra_.get(), ultrasonic_.get(), ir_.get());

        // ---- 超声来源解析 (v2.5): 先解析, 再决定是否把超声接进安全链 ----
        {
            const std::size_t pubs = this->count_publishers("ultrasonic");
            const bool hw_ok = ultrasonic_->is_hardware_available();
            ultrasonic_source_ = mechdog_ros::resolve_ultrasonic_source(
                ultrasonic_requested_, use_simulated_, hw_ok, pubs, allow_simulated_ultrasonic_);
            ultrasonic_enabled_ = (ultrasonic_source_ != "none");
            fusion_->set_ultrasonic_enabled(ultrasonic_enabled_);

            if (ultrasonic_source_ == "none") {
                RCLCPP_WARN(this->get_logger(),
                    "超声来源=none: 已从安全链**移除**超声 (不参与融合, 也不作悬崖判定)。"
                    "原因: 请求=%s 硬件(GPIO)=%d /ultrasonic 发布者=%zu —— "
                    "驱动回落的是**模拟随机数**(带 valid=true, 底部还有 5%% 概率造悬崖), "
                    "绝不允许它参与安全决策。恢复途径: 让 ultrasonic_node 发布 /ultrasonic, "
                    "或在 Pi 上编 USE_WIRINGPI 接真实 GPIO, 或(仅台架) "
                    "allow_simulated_ultrasonic:=true。",
                    ultrasonic_requested_.c_str(), static_cast<int>(hw_ok), pubs);
            } else if (ultrasonic_source_ == "simulated" && !use_simulated_) {
                RCLCPP_WARN(this->get_logger(),
                    "超声来源=simulated (真实模式下显式允许): 融合结果含**随机模拟数据**, "
                    "仅供台架验证, 不可据此判断真机安全行为。");
            }
            RCLCPP_INFO(this->get_logger(),
                "超声来源: %s (请求=%s GPIO=%d /ultrasonic 发布者=%zu 超时=%dms)",
                ultrasonic_source_.c_str(), ultrasonic_requested_.c_str(),
                static_cast<int>(hw_ok), pubs, ultrasonic_timeout_ms_);
        }
        planner_ = std::make_unique<PathPlanner>();

        // ---- 深度来源解析 + 启动 (v2.4) ----
        if (depth_source_ == "auto") {
#if defined(USE_ASTRA_SDK)
            depth_source_ = use_simulated_ ? "simulated" : "sdk";
#else
            depth_source_ = use_simulated_ ? "simulated" : "topic";
#endif
        }
        if (depth_source_ == "sdk") {
#if !defined(USE_ASTRA_SDK)
            RCLCPP_WARN(this->get_logger(),
                "depth_source=sdk 但本二进制未编译 Astra SDK (USE_ASTRA_SDK=OFF) → 深度恒无效; "
                "真机请用 depth_source:=topic");
#endif
        }
        if (depth_source_ == "topic" && !use_simulated_) {
            auto qos = rclcpp::SensorDataQoS();   // 图像话题惯例: best effort
            depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                depth_topic_, qos,
                [this](sensor_msgs::msg::Image::SharedPtr m) { on_depth_image(std::move(m)); });
            depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                depth_info_topic_, qos,
                [this](sensor_msgs::msg::CameraInfo::SharedPtr m) { on_depth_info(std::move(m)); });
            RCLCPP_INFO(this->get_logger(),
                "深度来源: ROS 话题 %s (内参 %s, 超时 %d ms) —— 不启动 Astra SDK 采集线程",
                depth_topic_.c_str(), depth_info_topic_.c_str(), depth_timeout_ms_);
        } else {
            if (depth_source_ == "topic") {
                RCLCPP_WARN(this->get_logger(),
                    "depth_source=topic 但 use_simulated=true → 模拟模式不订阅深度话题; "
                    "真机请设 use_simulated:=false depth_source:=topic");
            }
            astra_->start();   // simulated / sdk: 驱动自带采集线程
            RCLCPP_INFO(this->get_logger(), "深度来源: %s", depth_source_.c_str());
        }

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
            // R4 (REVIEW): 启动预热 —— 等底部线程首帧 (就绪即返, 有界超时; 超时也继续,
            //   后续 is_fall_risk() 仍 fail-closed 兜底)。放在首轮 fuse() 之前, 避免
            //   启动期 bottom 未就绪导致 cliff_detected=true -> 首帧必 STOP。
            mechdog_ros::wait_for_bottom_ready(
                *ultrasonic_, std::chrono::milliseconds(warmup_ms_));
            while (true) {
                auto t0 = std::chrono::steady_clock::now();
                // v2.5: 超声话题超时 → 把超声移出安全链。
                // 为什么不是"继续用旧帧": 不注入新鲜数据时, 算法库 read_all 会**回退到
                // 内部模拟随机数** (带 valid=true, 底部 5% 造悬崖), 那比"没有超声"更危险。
                if (ultrasonic_enabled_ && ultrasonic_source_ == "topic" && have_ultra_rx_) {
                    const auto age_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - last_ultra_rx_).count();
                    if (age_ms > ultrasonic_timeout_ms_) {
                        have_ultra_rx_ = false;
                        ultrasonic_enabled_ = false;
                        fusion_->set_ultrasonic_enabled(false);
                        RCLCPP_WARN(this->get_logger(),
                            "/ultrasonic 已 %ld ms 无数据 → 超声退出安全链 "
                            "(避免回落模拟随机数; 数据恢复后自动接回)",
                            static_cast<long>(age_ms));
                    }
                }
                // v2.4: 深度话题超时看门狗 —— 源断了就把帧标记失效 (fail-closed):
                //   不这么做的话, 最后一帧会被无限复用 (假"看得见"), 比看不见更危险。
                if (have_depth_rx_) {
                    const auto age_ms =
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - last_depth_rx_).count();
                    if (age_ms > depth_timeout_ms_) {
                        astra_->invalidate_frame();
                        have_depth_rx_ = false;   // 只失效一次, 避免每轮刷屏
                        RCLCPP_WARN(this->get_logger(),
                            "深度话题 %s 已 %ld ms 无数据 → 深度帧标记失效 "
                            "(fail-closed, 决策仅剩超声)",
                            depth_topic_.c_str(), static_cast<long>(age_ms));
                    }
                }
                auto result = fusion_->fuse();
                // ROS-3: 先判停止, 再决定是否写共享状态 (析构中不再访问成员)
                if (!fusion_running_.load()) break;
                // 路1: 近场地形状态变化时提示 (与核心仓 main.cpp 同口径, 便于真机排查)
                if (result.terrain_block_near || result.terrain_block_mid) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "路1 近场地形禁行: near=%d mid=%d 最近 x=%.2fm → 动作 %s",
                        static_cast<int>(result.terrain_block_near),
                        static_cast<int>(result.terrain_block_mid),
                        result.terrain_block_x_m,
                        action_to_str(result.recommended_action));
                }
                {
                    std::lock_guard<std::mutex> lock(result_mutex_);
                    latest_result_ = result;
                    have_result_ = true;
                    last_fusion_update_ = std::chrono::steady_clock::now();
                }
                // 近场感知: **每轮都更新** (与是否发布点云解耦) —— 路1 的避坑决策要靠它;
                // 原来只在 cloud_pub_ 存在时才跑分割, 会让"避坑"静默依赖可视化开关。
                update_perception();
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
            neg_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(neg_topic_, 5);
            RCLCPP_INFO(this->get_logger(),
                "近场点云已启用: topic=%s frame=%s 下采样步长=%d (配合 static TF %s -> base_link)",
                cloud_topic_.c_str(), cloud_frame_.c_str(), cloud_step_,
                cloud_frame_.c_str());
            RCLCPP_INFO(this->get_logger(),
                "负障碍检测已启用: topic=%s frame=%s (P1 地面分割, 落差阈值 %.2fm)",
                neg_topic_.c_str(), neg_frame_.c_str(), gseg_params_.cliff_drop_min);
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

        // 方案A: 订阅独立 ultrasonic_node 的 /ultrasonic, 注入算法库 UltrasonicArrayDriver
        // (替代算法库内部 GPIO 直读; 未收到/过期时算法库 read_all 回退内部读取, fail-safe)
        ultra_sub_ = this->create_subscription<mechdog_ultrasonic::msg::UltrasonicArray>(
            "ultrasonic", rclcpp::SensorDataQoS(),
            [this](const mechdog_ultrasonic::msg::UltrasonicArray::SharedPtr msg) {
                UltrasonicArrayData d;
                d.timestamp = rclcpp::Time(msg->stamp).seconds();
                auto to_reading = [](double cm, bool valid) {
                    UltrasonicReading r;
                    r.distance_cm = cm;
                    r.valid = valid;
                    return r;
                };
                d.front_left = to_reading(msg->front_left_cm, msg->front_left_valid);
                d.front_center = to_reading(msg->front_center_cm, msg->front_center_valid);
                d.front_right = to_reading(msg->front_right_cm, msg->front_right_valid);
                d.bottom = to_reading(msg->bottom_cm, msg->bottom_valid);
                ultrasonic_->inject_external_data(d);

                // v2.5: 记录新鲜度 + 恢复入口 —— 真数据到了就把超声重新接回安全链
                last_ultra_rx_ = std::chrono::steady_clock::now();
                have_ultra_rx_ = true;
                if (!ultrasonic_enabled_ && (ultrasonic_requested_ == "auto" ||
                                             ultrasonic_requested_ == "topic")) {
                    ultrasonic_source_ = "topic";
                    ultrasonic_enabled_ = true;
                    fusion_->set_ultrasonic_enabled(true);
                    RCLCPP_WARN(this->get_logger(),
                        "/ultrasonic 数据已到达 → 超声重新接入安全链 (来源=topic)");
                }
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
    // ---- v2.4 深度话题源 (depth_source=topic) ----
    // 深度图 → uint16(mm) → 注入驱动 (区域分析/环境判定走驱动内部同一链路)。
    // 支持 16UC1/mono16 (mm) 与 32FC1 (m); 其他编码节流告警后丢弃。
    void on_depth_image(sensor_msgs::msg::Image::SharedPtr msg) {
        const int w = static_cast<int>(msg->width);
        const int h = static_cast<int>(msg->height);
        if (w <= 0 || h <= 0) return;
        const size_t need = static_cast<size_t>(w) * static_cast<size_t>(h);
        std::vector<uint16_t> buf(need, 0);
        const std::string& enc = msg->encoding;
        if (enc == "16UC1" || enc == "mono16") {
            if (msg->data.size() < need * 2) return;
            std::memcpy(buf.data(), msg->data.data(), need * 2);
        } else if (enc == "32FC1") {
            if (msg->data.size() < need * 4) return;
            const float* src = reinterpret_cast<const float*>(msg->data.data());
            for (size_t i = 0; i < need; ++i) {
                const float m = src[i];
                buf[i] = (std::isfinite(m) && m > 0.0f)
                             ? static_cast<uint16_t>(m * 1000.0f + 0.5f) : 0;
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "深度话题编码不支持: %s (仅 16UC1/mono16/32FC1)", enc.c_str());
            return;
        }
        const double stamp_s = static_cast<double>(msg->header.stamp.sec)
                             + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        if (astra_->inject_depth_frame(buf, w, h, stamp_s)) {
            last_depth_rx_ = std::chrono::steady_clock::now();
            have_depth_rx_ = true;
        }
    }

    // 深度内参话题 → 替换 FOV 反推内参 (点云 / 2.5D / 路1 全部受益)
    void on_depth_info(sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
            cloud_K_.fx = msg->k[0];
            cloud_K_.fy = msg->k[4];
            cloud_K_.cx = msg->k[2];
            cloud_K_.cy = msg->k[5];
            if (!have_camera_info_) {
                have_camera_info_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "深度内参已从话题读取: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                    cloud_K_.fx, cloud_K_.fy, cloud_K_.cx, cloud_K_.cy);
            }
        }
    }

    // 近场感知更新 (每轮融合节拍调用, **与是否发布点云解耦**):
    //   depth → cloud(optical→link) → 下采样 → base 系 → 地面分割(P1) → 2.5D(P1.5)
    //   → 路1: fusion_->set_local_terrain(hm, seg)  近场走廊有坑/台阶 → STOP / 降速让开
    // 缓存 cloud_ds_link_ / cloud_base_ / seg_ 供 publish_pointcloud() 复用
    // (同一份云 + 同一份分割, 不重复跑 RANSAC, 也不出现"发布一套、决策另一套")。
    // 由融合线程调用 (独占相机读取, 无跨线程共享, 无需加锁)。
    void update_perception() {
        have_perception_ = false;
        AstraFrame frame = astra_->get_latest_frame();
        if (!frame.valid || frame.depth_map.empty() ||
            frame.depth_width <= 0 || frame.depth_height <= 0) {
            fusion_->clear_local_terrain();   // 无帧 → 不注入地形 (行为回到接入路1之前)
            return;  // 首帧未就绪 / 真机帧失效 (H1 同口径)
        }
        PointCloud cloud_opt, cloud_link;
        depth_to_cloud(frame.depth_map.data(), frame.depth_width,
                       frame.depth_height, cloud_K_, cloud_opt);
        transform_optical_to_link(cloud_opt, cloud_link);

        // 下采样 (发布与分割同源): 全量 30 万点在 Pi 上 10Hz 扛不住
        const size_t total = cloud_link.points.size();
        const size_t step = static_cast<size_t>(cloud_step_);
        cloud_ds_link_ = PointCloud{};
        cloud_ds_link_.seq = cloud_link.seq;
        cloud_ds_link_.stamp = cloud_link.stamp;
        cloud_ds_link_.frame_id = cloud_link.frame_id;
        cloud_ds_link_.points.reserve(total / step + 1);
        for (size_t i = 0; i < total; i += step) {
            cloud_ds_link_.points.push_back(cloud_link.points[i]);
        }
        if (cloud_ds_link_.points.empty()) {
            fusion_->clear_local_terrain();
            return;  // 全无效深度
        }

        transform_to_base(cloud_ds_link_, cloud_E_, cloud_base_);
        segment_ground(cloud_base_, gseg_params_, seg_);

        // ---- 路1: 近场地形 → 融合决策 (P1 负障碍点 + P1.5 禁行格) ----
        HeightMap25Config hcfg;
        HeightMap25Result hm;
        build_heightmap_25(cloud_base_, seg_, hcfg, hm);
        fusion_->set_local_terrain(hm, seg_);

        // 诊断 (真机排查): 平面 / 2.5D / 负障碍 状态。
        // 没有这条日志时, "路1 一声不吭"只能靠猜 —— fail-closed 静默是安全设计,
        // 但排查时必须能看到"为什么静默"(无平面? 走廊空? 还是根本没跑到这里)。
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "感知: 点云=%zu 平面valid=%d tilt=%.2f° h0=%.3fm 内点=%zu neg=%zu | 2.5D %s",
            cloud_base_.points.size(), static_cast<int>(seg_.plane.valid),
            std::acos(std::min(1.0, std::max(-1.0, static_cast<double>(seg_.plane.nz)))) *
                180.0 / 3.14159265358979323846,
            static_cast<double>(seg_.plane.height_at_origin()),
            static_cast<size_t>(seg_.plane.inliers), seg_.negative_points.size(),
            hm.valid ? hm.stats().c_str() : "invalid (无平面→fail-closed, 路1 静默)");

        have_perception_ = true;
    }

    // 近场点云 + 负障碍发布 (使用 update_perception() 的缓存; 由融合线程调用,
    // rclcpp publish 线程安全; stamp 用发布时刻 —— 静态 TF 对任意时刻有效,
    // Nav2 voxel_layer 按"最新观测"消费)
    void publish_pointcloud() {
        if (!have_perception_ || cloud_ds_link_.points.empty()) return;
        auto stamp = this->now();
        cloud_pub_->publish(marshal_xyz(cloud_ds_link_.points, cloud_frame_, stamp));

        // 近场负障碍 (P1): base_link 系地面分割 → 坑/下行台阶标记点
        // 注意: 分割用降采样云 (与发布同源), 全量 30 万点在 Pi 上 10Hz 扛不住
        if (neg_pub_) {
            const GroundSegResult& seg = seg_;
            if (!seg.negative_points.empty()) {
                neg_pub_->publish(
                    marshal_xyz(seg.negative_points, neg_frame_, stamp));
            } else if (!seg.plane.valid && !use_simulated_) {
                // 真机找不到地面平面 (俯仰/装高参数不对? 或镜头没对着地面) —— 节流提醒
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                    "地面平面拟合失败, 负障碍检测停用 (检查相机外参/ground_prior)");
            }
            // 模拟模式: 仿真数据是纯墙面无地面, plane 恒 invalid, 静默跳过
        }
    }

    // xyz 点列 -> PointCloud2 (x/y/z float32 + 4 字节 padding, point_step=16)
    static sensor_msgs::msg::PointCloud2 marshal_xyz(
        const std::vector<Point3D>& pts, const std::string& frame,
        const rclcpp::Time& stamp) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(pts.size());
        msg.is_dense = true;
        msg.is_bigendian = false;
        msg.point_step = 16;
        msg.row_step = msg.point_step * msg.width;
        sensor_msgs::msg::PointField field;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        field.name = "x"; field.offset = 0; msg.fields.push_back(field);
        field.name = "y"; field.offset = 4; msg.fields.push_back(field);
        field.name = "z"; field.offset = 8; msg.fields.push_back(field);
        msg.data.reserve(pts.size() * 16);
        for (const auto& p : pts) {
            const float xyz[4] = {static_cast<float>(p.x), static_cast<float>(p.y),
                                  static_cast<float>(p.z), 0.0f};
            const auto* bytes = reinterpret_cast<const uint8_t*>(xyz);
            msg.data.insert(msg.data.end(), bytes, bytes + sizeof(xyz));
        }
        return msg;
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr neg_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<mechdog_ultrasonic::msg::UltrasonicArray>::SharedPtr ultra_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool use_simulated_ = true;
    std::string cmd_vel_topic_ = "/unsafe/cmd_vel";
    // R4 (REVIEW): 启动预热时长 (ms), 见参数声明处
    int warmup_ms_ = 250;
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

    // 负障碍 (P1): 地面分割参数取算法库默认 (GroundSegConfig), 手持实验改 config.h
    std::string neg_topic_ = "/mechdog/negative_obstacles";
    std::string neg_frame_ = "base_link";
    CameraExtrinsics cloud_E_;    // 外参占位值 (装机量测后与 launch TF 同步更新)
    GroundSegParams gseg_params_;

    // 路1: 近场感知缓存 (由 update_perception() 每轮刷新, publish_pointcloud() 复用)
    //   口径: 与发布同一份下采样云 + 同一份地面分割结果 (避免重复 RANSAC / 双份真相)
    PointCloud cloud_ds_link_;    // 发布用 (camera_link 系)
    PointCloud cloud_base_;       // 分割/2.5D 用 (base_link 系)
    GroundSegResult seg_;         // P1 地面分割 (含 negative_points)
    bool have_perception_ = false;

    // ---- 相机几何 (v2.6) ----
    double cloud_x_ = 0.12;              // 算法侧外参 (与 launch 静态 TF 同源)
    double cloud_z_ = 0.18;
    double cloud_pitch_rad_ = 0.2617994; // 15°
    double camera_height_m_ = -1.0;      // 相机镜头离地高 (>0 时推导地面高度先验)
    double prior_window_m_ = -1.0;       // 地面高度先验半带宽 (<=0 = 用仓库默认)

    // ---- 超声来源 (v2.5): 拒绝把模拟随机数喂进安全链 ----
    std::string ultrasonic_requested_ = "auto";   // 用户请求值
    std::string ultrasonic_source_ = "auto";      // 解析后的实际来源
    bool        ultrasonic_enabled_ = true;       // 是否接在安全链上
    bool        allow_simulated_ultrasonic_ = false;
    int         ultrasonic_timeout_ms_ = 500;
    bool        have_ultra_rx_ = false;
    std::chrono::steady_clock::time_point last_ultra_rx_{};

    // 深度来源 (v2.4): depth_source=topic 时的订阅与新鲜度状态
    std::string depth_source_ = "auto";
    std::string depth_topic_ = "/camera/depth/image_raw";
    std::string depth_info_topic_ = "/camera/depth/camera_info";
    int  depth_timeout_ms_ = 500;
    bool have_depth_rx_ = false;       // 收到过话题帧 (供超时看门狗判断)
    bool have_camera_info_ = false;
    std::chrono::steady_clock::time_point last_depth_rx_{};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

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
