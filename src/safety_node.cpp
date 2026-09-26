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
#include <array>                            // v2.8.2 汇报可视化用
#include <std_msgs/msg/string.hpp>          // v2.8.2 状态栏
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
        // v2.8.3: 网格收敛到视场楔形 (只改统计口径, 判据/格数不变)
        grid_wedge_only_ = this->declare_parameter("grid_wedge_only", true);
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
        // v2.8: 安装横滚 —— "镜头平行地面"是硬规格(pitch=0), 横滚由**竖墙标定**给出
        //   (wall_calib: 竖直面法向的仰角 = roll 偏差; 台架实测 +2.35° ⇒ -0.041 rad)
        const double cloud_roll_rad = this->declare_parameter("cloud_roll_rad", 0.0);
        // v2.9.6: 安装偏航 —— 三参数标定用。注意: 单个水平面**观测不出 yaw**
        //   (绕竖轴旋转不改变平面倾角), 故 yaw 只能由竖直面/已知朝向给出;
        //   本参数先暴露出来(默认 0), 等机械装夹对齐机体后置 0 即可。
        const double cloud_yaw_rad  = this->declare_parameter("cloud_yaw_rad", 0.0);
        camera_height_m_ = this->declare_parameter("camera_height_m", -1.0);
        prior_window_m_  = this->declare_parameter("ground_prior_window", -1.0);
        // v2.9.4 配置护栏: 地面先验半带宽是**安全参数** (与镜头离地高强相关)。放宽它
        //   等于允许"错误高度的平面"被当作地面 —— 实测事故: 台架传 0.6 时, 椅面(h0=-0.34)
        //   被判为地面 ⇒ 真正的地板反被当成坑(down=20)、近场走廊失效(near=0)。
        //   装机路径必须保持仓库默认 0.10; 台架请改用 camera_height_m:=<实测镜头高> 推导。
        if (prior_window_m_ > 0.25) {
            RCLCPP_WARN(this->get_logger(),
                "ground_prior_window=%.2fm 明显放宽 (>0.25m) —— 允许错误高度的平面被当作地面; "
                "台架请改用 camera_height_m:=<实测镜头高> 推导先验, 装机应保持默认 0.10m",
                prior_window_m_);
        }
        // v2.7: 地面提取方法 cell(确定性格最小拟合)|ransac; 默认 ransac = 与历史行为一致
        this->declare_parameter("ground_fit_method", std::string("ransac"));
        // 切换点: 必须在构造函数里赋值 (成员声明区不能写语句 —— 踩过, 构建报 code 2)
        gseg_params_.use_cell_min_fit =
            (this->get_parameter("ground_fit_method").as_string() == "cell");
        // v2.9.16 (2026-09-25 师兄口径): cell 成功时跳过 RANSAC(省 11~16ms/帧 且保留 cell 精度);
        //   置 false 回到旧行为(供真机 A/B / 排查)。
        this->declare_parameter("cell_skip_ransac", true);
        gseg_params_.cell_skip_ransac = this->get_parameter("cell_skip_ransac").as_bool();
        // v2.9.18 (N8): 护栏提示 —— 该开关只在 ground_fit_method=cell 时有意义
        if (!gseg_params_.use_cell_min_fit && gseg_params_.cell_skip_ransac) {
            RCLCPP_INFO(this->get_logger(),
                "cell_skip_ransac=true 但 ground_fit_method≠cell ⇒ 该开关不生效 (cell 未启用)");
        }
        // v2.9.17 (2026-09-25 师兄口径 ②): 深度可用性"时域"判据 ——
        //   连续 N 轮拿不到可用深度(无帧 / 质量守门 / 点云为空) ⇒ 标记"深度降级"并在日志与
        //   /safety/status_text 显式上报; 拿到好帧后自动解除。
        //   单帧阈值(25%/300)与 fail-closed / 路1 abstain 口径一律不变;
        //   v2.9.20 (S1, N2): 降级期间追加动作层兜底 —— 前进限速≤SLOW + 三级反应线收紧
        //   (10/25/50 → 20/40/70cm), 由 degraded_policy 开关 (默认 true; 数值待会签)。
        this->declare_parameter("depth_bad_streak_n", 3);
        {
            // v2.9.18 (N8): 护栏 —— <1 时按 1 处理并告警 (否则"首个坏帧即上报"与原意不符)
            const int streak_raw = static_cast<int>(this->get_parameter("depth_bad_streak_n").as_int());
            if (streak_raw < 1) {
                RCLCPP_WARN(this->get_logger(),
                    "depth_bad_streak_n=%d (<1) ⇒ 按 1 处理; 建议 >=3", streak_raw);
            }
            depth_bad_streak_n_ = std::max(1, streak_raw);
        }
        // v2.9.20 (S1, N2): 降级链开关 —— true: 降级期"前进限速≤SLOW + 反应线 20/40/70cm";
        //   false: 回 v2.9.19 "仅上报"旧行为 (供真机 A/B 与师兄口径复核)。
        degraded_policy_ = this->declare_parameter("degraded_policy", true);
        RCLCPP_INFO(this->get_logger(),
            "退化期降级链(S1): %s (前进限速≤SLOW / 阈值20-40-70cm; degraded_policy:=false 关闭)",
            degraded_policy_ ? "启用" : "关闭(仅上报)");
        const double prior_override = this->declare_parameter("ground_prior_z", -999.0);

        cloud_E_.x     = cloud_x_;
        cloud_E_.y     = 0.0;
        cloud_E_.z     = cloud_z_;
        cloud_E_.roll  = cloud_roll_rad;   // v2.8 竖墙标定值
        cloud_E_.pitch = cloud_pitch_rad_;
        cloud_E_.yaw   = cloud_yaw_rad;   // v2.9.6 三参数标定 (默认 0)
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
        // v2.9.15 (OFFLINE_TODO #9): 汇报用小图 —— 窗口不再订阅 614KB 原始深度。
        //   节点侧 1/2 抽点后发布 16UC1 320x240(约153KB/帧); 窗口此前自己就在做同样的
        //   1/2 抽点 ⇒ 显示结果逐像素不变。纯显示通道, 不参与任何判定。
        publish_depth_small_ = this->declare_parameter("publish_depth_small", true);
        if (publish_depth_small_) {
            depth_small_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/safety/depth_small", 1);
        }
        // v2.9.18 (N9): status_text 提前到构造期创建 —— 启动期深度全坏时该话题也要被
        //   `ros2 topic list` 看到, 且降级文案(经 note_depth_bad)可以立即发布。
        status_text_pub_ = this->create_publisher<std_msgs::msg::String>("/safety/status_text", 1);
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
            // v2.9.19 (复审批 B4): topic 来源在**首帧到达前一律不接入安全链**。
            //   旧版启动即 enabled=true, 而超时看门狗要"收到过首帧"才武装(have_ultra_rx_)
            //   ⇒ "有发布者但从未发帧"的窗口里 fuse() 消费的是算法库 read_all() 回落的
            //   内部模拟随机数(valid=true, 底部 5% 造悬崖) —— 安全链里绝不能有这种数据。
            //   首帧到达后走 ultra_sub_ 回调里的现成恢复路径(超声重新接入安全链)。
            if (ultrasonic_source_ == "topic") {
                ultrasonic_enabled_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "超声来源=topic: 等待 /ultrasonic 首帧 —— 到达前超声不参与安全链 "
                    "(防未注入时回落内部模拟随机数)");
            }
            fusion_->set_ultrasonic_enabled(ultrasonic_enabled_);
            // v2.9.19 (B4): 驱动注入超时与节点看门狗同源(旧版驱动 1.0s / 节点 500ms)
            ultrasonic_->set_inject_timeout_sec(
                static_cast<double>(ultrasonic_timeout_ms_) / 1000.0);
            last_ultra_rx_ = std::chrono::steady_clock::now();   // 首帧等待计时起点

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
                // v2.9.19 (B4): 发布者存在但从未发过首帧 ⇒ 5s 时 WARN 一次 (现场可诊断)。
                //   只针对"从未收到"; 收到过后再静默由上面的超时看门狗负责。
                if (!ultrasonic_enabled_ && ultrasonic_source_ == "topic" &&
                    !have_ultra_rx_ && !ultra_ever_rx_) {
                    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - last_ultra_rx_).count();
                    if (age_ms > 5000 && !topic_wait_warned_) {
                        topic_wait_warned_ = true;
                        RCLCPP_WARN(this->get_logger(),
                            "超声来源=topic 已 %.1f s 未收到任何 /ultrasonic 消息 —— 请检查"
                            "发布端(ultrasonic_node/桥)是否在发; 期间超声不参与安全链",
                            static_cast<double>(age_ms) / 1000.0);
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
                const auto tU0 = std::chrono::steady_clock::now();
                update_perception();
                // 近场点云: 跟随融合节拍发布 (融合线程独占相机读取; rclcpp publish 线程安全)
                if (cloud_pub_) {
                    publish_pointcloud();
                }
                // RGB 回传: 节流到目标帧率后发布 Astra 彩色帧 (同一驱动实例, 免抢相机)
                if (rgb_pub_) {
                    publish_rgb_if_due();
                }
                const auto tU1 = std::chrono::steady_clock::now();
                {   // v2.9.11 分段计时(限流 2s; 仅观测, 不参与任何决策)
                    const double total = std::chrono::duration<double, std::milli>(tU1 - tU0).count();
                    const double inner = ms_gate_ + ms_backproj_ + ms_to_base_ + ms_seg_ + ms_heightmap_;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[计时段] 主线程: 解码=%.1f 驱动注入=%.1f | 感知: 门槛=%.1f 反投影=%.1f 到base=%.1f 分割=%.1f 高度图=%.1f | 感知小计=%.1f 其余(可视化/发布)=%.1f 总=%.1f ms",
                        ms_decode_, ms_inject_, ms_gate_, ms_backproj_, ms_to_base_, ms_seg_,
                        ms_heightmap_, inner, total - inner, total);
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
                ultra_ever_rx_ = true;   // v2.9.19 (B4)
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
    // v2.9.11 计时(仅观测): 主线程 解码拷贝 / 驱动注入 两段
    void on_depth_image(sensor_msgs::msg::Image::SharedPtr msg) {
        const auto t_img0 = std::chrono::steady_clock::now();
        const int w = static_cast<int>(msg->width);
        const int h = static_cast<int>(msg->height);
        if (w <= 0 || h <= 0) return;
        const size_t need = static_cast<size_t>(w) * static_cast<size_t>(h);
        std::vector<uint16_t> buf(need, 0);
        const std::string& enc = msg->encoding;
        // v2.9.18 (B10): 按 msg->step (行步长) 逐行拷贝 —— 旧实现 memcpy(need*2) 假定
        //   无行填充, 一旦驱动带 padding (step > width*2) 会把整幅图错位/错读。
        if (enc == "16UC1" || enc == "mono16") {
            const size_t row_bytes = static_cast<size_t>(w) * 2;
            const size_t step = (msg->step >= row_bytes) ? static_cast<size_t>(msg->step) : row_bytes;
            if (msg->data.size() < step * (static_cast<size_t>(h) - 1) + row_bytes) return;
            if (step == row_bytes) {
                std::memcpy(buf.data(), msg->data.data(), row_bytes * static_cast<size_t>(h));
            } else {
                for (int y = 0; y < h; ++y) {
                    std::memcpy(buf.data() + static_cast<size_t>(y) * static_cast<size_t>(w),
                                msg->data.data() + static_cast<size_t>(y) * step, row_bytes);
                }
            }
        } else if (enc == "32FC1") {
            const size_t row_bytes = static_cast<size_t>(w) * 4;
            const size_t step = (msg->step >= row_bytes) ? static_cast<size_t>(msg->step) : row_bytes;
            if (msg->data.size() < step * (static_cast<size_t>(h) - 1) + row_bytes) return;
            for (int y = 0; y < h; ++y) {
                const float* src = reinterpret_cast<const float*>(
                    msg->data.data() + static_cast<size_t>(y) * step);
                uint16_t* dst = buf.data() + static_cast<size_t>(y) * static_cast<size_t>(w);
                for (int x = 0; x < w; ++x) {
                    const float m = src[x];
                    dst[x] = (std::isfinite(m) && m > 0.0f)
                                 ? static_cast<uint16_t>(m * 1000.0f + 0.5f) : 0;
                }
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "深度话题编码不支持: %s (仅 16UC1/mono16/32FC1)", enc.c_str());
            return;
        }
        // v2.9.15: 顺手发一份 1/2 抽点小图给汇报窗口(在注入之前, 不影响注入结果)
        publish_depth_small(buf, w, h, msg->header);
        const double stamp_s = static_cast<double>(msg->header.stamp.sec)
                             + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
        const auto t_inj0 = std::chrono::steady_clock::now();
        if (astra_->inject_depth_frame(buf, w, h, stamp_s)) {
            last_depth_rx_ = std::chrono::steady_clock::now();
            have_depth_rx_ = true;
        }
        const auto t_inj1 = std::chrono::steady_clock::now();
        // 主线程耗时(供感知线程一并打印; 仅日志用, 允许无锁读取)
        ms_decode_ = std::chrono::duration<double, std::milli>(t_inj0 - t_img0).count();
        ms_inject_ = std::chrono::duration<double, std::milli>(t_inj1 - t_inj0).count();
    }

    // v2.9.15 (OFFLINE_TODO #9): 汇报用小图 —— 1/2 抽点(640x480→320x240)后发布。
    // 目的: 汇报窗口不再订阅 614KB 原始深度并做全分辨率 float32 转换(此前窗口 CPU 大户)。
    // 显示等价性: 窗口原本自己在 Python 里做同样的 1/2 抽点 ⇒ 画面逐像素不变。
    // 只做整数抽点, 不新建全分辨率缓冲 ⇒ 成本(估算, 待真机实测) 约 0.2~0.5ms/帧。
    void publish_depth_small(const std::vector<uint16_t>& buf, int w, int h,
                             const std_msgs::msg::Header& header) {
        if (!depth_small_pub_) return;
        const int w2 = w / 2, h2 = h / 2;
        if (w2 <= 0 || h2 <= 0) return;
        auto img = std::make_unique<sensor_msgs::msg::Image>();
        img->header = header;
        img->height = static_cast<uint32_t>(h2);
        img->width  = static_cast<uint32_t>(w2);
        img->encoding = "16UC1";
        img->is_bigendian = 0;
        img->step = static_cast<uint32_t>(w2) * 2u;
        img->data.resize(static_cast<size_t>(w2) * static_cast<size_t>(h2) * 2u);
        uint16_t* dst = reinterpret_cast<uint16_t*>(img->data.data());
        for (int y = 0; y < h2; ++y) {
            const uint16_t* srow = buf.data() + static_cast<size_t>(y * 2) * static_cast<size_t>(w);
            uint16_t* drow = dst + static_cast<size_t>(y) * static_cast<size_t>(w2);
            for (int x = 0; x < w2; ++x) {
                drow[x] = srow[x * 2];
            }
        }
        depth_small_pub_->publish(std::move(img));
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

    // v2.9.17 (口径 ②): 降级时**主动**发一条状态文案 —— 坏帧走早退路径, 否则窗口永远显示
    //   最后一次"好"文案(现场看起来一切正常), 正是要消灭的"静默"。
    void publish_depth_degraded_status(const char* why) {
        if (!status_text_pub_)
            status_text_pub_ = this->create_publisher<std_msgs::msg::String>("/safety/status_text", 1);
        std_msgs::msg::String s;
        char b[256];
        std::snprintf(b, sizeof(b),
            "DEPTH DEGRADED: 连续 %d 轮无可用深度 (%.1fs, 最近原因=%s) -- 本轮不注入地形(fail-closed); "
            "查相机/USB/光照, 非安装角%s",
            depth_bad_streak_, this->now().seconds() - depth_bad_t0_, why,
            degraded_policy_ ? " | 降级链:限速+阈值20-40-70cm" : "");
        s.data = b;
        status_text_pub_->publish(s);
    }

    // v2.9.17 (口径 ②) / v2.9.20 (S1, N2): 记录"本轮拿不到可用深度"。
    //   单帧阈值与 fail-closed 口径不变; 达阈值后按 degraded_policy 启用降级链 (限速/阈值收紧) 或仅上报。
    void note_depth_bad(int reason) {   // 0=无帧 1=质量守门 2=点云为空
        // v2.9.18 (N9): 启动期(从未见过好帧)也计入 —— "相机没插/USB 没起/驱动失败"恰是最
        //   需要告警的场景; 旧版这里直接 return, 把它整个排除了。防启动瞬态误报: 未见过好帧
        //   时首次升级门槛提高为 max(30, depth_bad_streak_n_) 轮(≈3s@10Hz)。不改单帧判定。
        depth_last_reason_ = reason;
        if (depth_bad_streak_ == 0) depth_bad_t0_ = this->now().seconds();
        ++depth_bad_streak_;
        const int eff_n = depth_ever_good_
            ? std::max(1, depth_bad_streak_n_)
            : std::max(30, depth_bad_streak_n_);
        const char* why = (reason == 0) ? "no frame" : (reason == 2) ? "empty cloud" : "quality gate";
        if (depth_bad_streak_ == eff_n && !depth_degraded_) {
            depth_degraded_ = true;
            // v2.9.20 (S1, N2): 降级链生效 —— 上行"前进限速≤SLOW + 三级反应线收紧"
            //   (degraded_policy:=false 时仅上报, 行为与 v2.9.19 一致)。
            if (degraded_policy_) fusion_->set_depth_degraded(true);
            const char* chain = degraded_policy_
                ? "降级链已启用 (前进限速≤SLOW / 阈值20-40-70cm)"
                : "降级链未启用 (degraded_policy:=false, 仅上报)";
            if (!depth_ever_good_) {
                RCLCPP_WARN(this->get_logger(),
                    "深度降级(启动期): 启动以来连续 %d 轮拿不到可用深度 (%.2fs, 最近原因=%s) —— "
                    "疑似相机/USB/驱动未就绪 (或镜头盖未摘); \"从未见过好帧\"不是正常状态; %s",
                    depth_bad_streak_, this->now().seconds() - depth_bad_t0_, why, chain);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "深度降级: 连续 %d 轮拿不到可用深度 (%.2fs, 最近原因=%s) ⇒ 路1/2.5D 持续 abstain; %s; "
                    "排查方向 = 相机/USB/光照, 不是安装角。好帧自动解除 (参数 depth_bad_streak_n=%d)",
                    depth_bad_streak_, this->now().seconds() - depth_bad_t0_, why, chain, depth_bad_streak_n_);
            }
        }
        if (depth_degraded_) {
            // 降级期间每轮刷新一次(1Hz 节流), 保证窗口里看到的就是"当前"状态
            const double now_s = this->now().seconds();
            if (now_s - last_degraded_status_s_ >= 1.0) {
                last_degraded_status_s_ = now_s;
                publish_depth_degraded_status(why);
            }
        }
    }
    void note_depth_good() {
        const bool first_ever = !depth_ever_good_;
        depth_ever_good_ = true;
        if (depth_bad_streak_ == 0) return;
        const double dt = this->now().seconds() - depth_bad_t0_;
        if (depth_degraded_) {
            if (degraded_policy_) fusion_->set_depth_degraded(false);   // S1: 降级解除, 限速/阈值复原
            RCLCPP_WARN(this->get_logger(),
                "深度恢复: 此前连续 %d 轮不可用 (%.2fs) ⇒ 降级解除%s", depth_bad_streak_, dt,
                degraded_policy_ ? ", 限速/阈值复原" : "");
        } else if (first_ever) {
            RCLCPP_INFO(this->get_logger(),
                "深度流就绪: 启动期 %d 轮无可用深度 (%.2fs) 后拿到首帧", depth_bad_streak_, dt);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "深度瞬时坏帧已恢复 (连续 %d 轮, 未达降级阈值 %d)", depth_bad_streak_, depth_bad_streak_n_);
        }
        depth_bad_streak_ = 0;
        depth_degraded_   = false;
    }

    // 近场感知更新 (每轮融合节拍调用, **与是否发布点云解耦**):
    //   depth → cloud(optical→link) → 下采样 → base 系 → 地面分割(P1) → 2.5D(P1.5)
    //   → 路1: fusion_->set_local_terrain(hm, seg)  近场走廊有坑/台阶 → STOP / 降速让开
    // 缓存 cloud_ds_link_ / cloud_base_ / seg_ 供 publish_pointcloud() 复用
    // (同一份云 + 同一份分割, 不重复跑 RANSAC, 也不出现"发布一套、决策另一套")。
    // 由融合线程调用 (独占相机读取, 无跨线程共享, 无需加锁)。
    void update_perception() {
        const auto tS0 = std::chrono::steady_clock::now();
        have_perception_ = false;
        depth_gate_hit_ = false;   // v2.9.12: 每轮复位, 只有本轮真被拦下才置位
        AstraFrame frame = astra_->get_latest_frame();
        if (!frame.valid || frame.depth_map.empty() ||
            frame.depth_width <= 0 || frame.depth_height <= 0) {
            fusion_->clear_local_terrain();   // 无帧 → 不注入地形 (行为回到接入路1之前)
            note_depth_bad(0);   // v2.9.17 口径②: 计入"连续坏帧"
            return;  // 首帧未就绪 / 真机帧失效 (H1 同口径)
        }
        // v2.9.3 深度质量守门 (方案 §4.3): 坏数据比没数据更危险 —— 实机事故中深度全 0 帧
        // 造出 down=22~27 的假坑, 顶层据此 13/13 全 STOP。质量不达标 ⇒ 本轮**不注入地形**
        // (2.5D 判未就绪 / 路1 abstain), 决策回落距离阶梯与超声; 悬崖/失明/超声层不受影响。
        {
            // v2.9.10 提速: 有效像素数已由 AstraProDriver::inject_depth_frame 在**合并后的单遍扫描**
            //   里精确算出(非抽样) ⇒ 此处不再重扫 30.7 万像素。等价性依据:
            //   ① 帧是"局部构建完整 → 加锁一次性 std::move 替换", 消费者读到的必为已清洗帧;
            //   ② 清洗后 `!= 0` 与"在 [MIN,MAX] 量程内"完全等价;
            //   ③ 离线 A/B 工装 test_astra_equiv.cpp 在 6 种图案上逐字段对照 = 逐位一致。
            const size_t dn = frame.depth_map.size();
            const size_t dv = frame.valid_pixel_count;
            const double vr = (dn > 0) ? static_cast<double>(dv) / static_cast<double>(dn) : 0.0;
            DepthQualityIssue qissue = DepthQualityIssue::Ok;
            if (!depth_quality_ok(vr, static_cast<int>(dv), qissue)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "深度质量未就绪 (issue=%d, valid=%.1f%% / %zu px) ⇒ 本轮不注入地形 (路1 abstain)",
                    static_cast<int>(qissue), vr * 100.0, dv);
                depth_gate_hit_ = true;   // v2.9.12: 发布侧据此说"深度坏帧", 不再误导为"没地面"
                depth_gate_vr_  = vr;
                depth_gate_px_  = dv;
                fusion_->clear_local_terrain();   // 未就绪 ⇒ 路1 不表态 (行为回到接入路1之前)
                note_depth_bad(1);   // v2.9.17 口径②
                return;
            }
        }

        // v2.9.9 提速 (真机实测根因): 原先先做"全量反投影(640×480=30.7 万像素)"、再对**全量点云**
        //   做 optical→link 变换 —— 两次全分辨率遍历 ⇒ ~42ms/帧 ⇒ 节点 CPU 81%(19Hz 深度流)。
        //   改为**直接按步长反投影**: 只算 1/step 的像素, 之后所有变换只在这个小子集上做 ⇒ 预计 ~5ms/帧。
        //   等价性: 新点集的每个点, 与老实现同像素算出的点**逐位相同**(子集关系);
        //   test_strided_backprojection_subset 覆盖, 并用变异测试确认它真的会失败(failed=1)。
        const size_t step = static_cast<size_t>((cloud_step_ > 1) ? cloud_step_ : 1);
        const auto tS1 = std::chrono::steady_clock::now();
        ms_gate_ = std::chrono::duration<double, std::milli>(tS1 - tS0).count();
        PointCloud cloud_ds_opt;
        depth_to_cloud_strided(frame.depth_map.data(), frame.depth_width,
                               frame.depth_height, cloud_K_,
                               static_cast<int>(step), cloud_ds_opt);
        // 发布用的 camera_link 系点云: 由同一份小点云转一次得到 (不再做全量变换)
        cloud_ds_link_ = PointCloud{};
        transform_optical_to_link(cloud_ds_opt, cloud_ds_link_);
        if (cloud_ds_opt.points.empty()) {
            depth_gate_hit_ = true;   // v2.9.12: 全无效深度 ⇒ 数据坏, 非构图问题
            fusion_->clear_local_terrain();
            note_depth_bad(2);   // v2.9.17 口径②
            return;  // 全无效深度
        }

        // v2.8.1 修复(关键): transform_to_base 内部已含 optical→link (point_cloud.cpp:112),
        //   所以**不能**把 link 系点云再喂给它 —— 那会转两次, base 系前后/上下颠倒。
        //   实测症状: 地板 z 最低只有 -0.56m(相机离地0.9m)、known 恒 55/4848、倾角乱跳、trav 恒 0。
        //   这里另做一份"光学系下采样"专供感知; 发布用的 cloud_ds_link_ 保持原样(逐点等价)。
        // (v2.9.9: cloud_ds_opt 已在上方按步长直接构建, 此处只做一次变换)
        const auto tS2 = std::chrono::steady_clock::now();
        ms_backproj_ = std::chrono::duration<double, std::milli>(tS2 - tS1).count();
        transform_to_base(cloud_ds_opt, cloud_E_, cloud_base_);
        const auto tS3 = std::chrono::steady_clock::now();
        ms_to_base_ = std::chrono::duration<double, std::milli>(tS3 - tS2).count();
        segment_ground(cloud_base_, gseg_params_, seg_);

        // ---- 路1: 近场地形 → 融合决策 (P1 负障碍点 + P1.5 禁行格) ----
        HeightMap25Config hcfg;
        // v2.8.3: 网格收敛到视场楔形 —— 只改"未知率"的统计口径, 判据/可行性格数不变。
        //   实机同帧验证(real_test_16, 几何修复后): 楔形内 known 801/2228,
        //   **楔形外被排除的 known = 0**(一个都不丢) ⇒ 分母从 4848 收到 2228 更诚实。
        //   (几何修复前曾测得 0/2228, 那是 base 系双转 bug 的症状, 非楔形问题)
        hcfg.wedge_only = grid_wedge_only_;
        HeightMap25Result hm;
        const auto tS4 = std::chrono::steady_clock::now();
        ms_seg_ = std::chrono::duration<double, std::milli>(tS4 - tS3).count();
        build_heightmap_25(cloud_base_, seg_, hcfg, hm);
        const auto tS5 = std::chrono::steady_clock::now();
        ms_heightmap_ = std::chrono::duration<double, std::milli>(tS5 - tS4).count();
        fusion_->set_local_terrain(hm, seg_);
        note_depth_good();   // v2.9.17 口径②: 本轮拿到可用深度

        // ---- v2.8.2 汇报用可视化: 发布 2.5D 可行度图 + 状态文本 ----
        //   (画面上的可行度图 = 节点真实决策依据, 不是事后重算, 汇报口径最硬)
        // v2.9.7: **每帧都发布** —— 原实现只在“平面有效”时才发, 无平面(如镜头对着墙)时
        //   话题停更 ⇒ 汇报窗口那一格没有新帧可画 ⇒ 看起来像“卡死/卡顿”(实测被误判成 CPU 问题,
        //   追查近一小时)。现在无论平面是否有效都发一张图, 话题节奏恒定;
        //   无平面时发“全未知”图, 并在状态文本里写明原因。
        const bool plane_ok = hm.valid && seg_.plane.valid;
        const bool grid_ok = (hm.cols > 0 && hm.rows > 0);
        // v2.9.8: 无平面时 build_heightmap_25 直接 return ⇒ 网格为空(fail-closed 是决策侧的正确行为),
        //   但发布侧不能因此沉默: 否则汇报窗口那一格没有新帧可画, 看起来像卡死。
        //   实测: 对着墙面/晃动时 /safety/terrain_map 90 秒零消息, 而深度仍 30Hz;
        //   节点进程 State=S/wchan=futex_wait(未卡死) ⇒ 就是没走到发布。
        //   现在按配置尺寸自造"全未知"图, 话题节奏恒定。
        const int cols = grid_ok ? hm.cols
            : static_cast<int>((hcfg.max_x_m - hcfg.min_x_m) / hcfg.cell_size) + 1;
        const int rows = grid_ok ? hm.rows
            : static_cast<int>(2.0 * hcfg.y_half_m / hcfg.cell_size) + 1;
        if (cols > 0 && rows > 0) {
            static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tpub;
            if (!tpub) tpub = this->create_publisher<sensor_msgs::msg::Image>("/safety/terrain_map", 1);
            if (!status_text_pub_)   // v2.9.17: 改为成员 —— 降级时(早退路径)也要能主动发文案
                status_text_pub_ = this->create_publisher<std_msgs::msg::String>("/safety/status_text", 1);
            auto& spub = status_text_pub_;
            static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grpub;
            if (!grpub) grpub = this->create_publisher<sensor_msgs::msg::Image>("/safety/terrain_grid", 1);
            const int sc = 4;                       // 放大倍数
            const int W = rows * sc, H = cols * sc;   // 横轴=x(前进), 纵轴=y
            sensor_msgs::msg::Image img;
            img.header.stamp = this->now();
            img.header.frame_id = "base_link";
            img.height = H; img.width = W; img.encoding = "rgb8";
            img.is_bigendian = 0; img.step = W * 3;
            img.data.assign(static_cast<size_t>(W) * H * 3, 0);
            auto color_of = [](CellFlag f) -> std::array<uint8_t, 3> {
                switch (f) {
                    case CellFlag::Traversable: return {40, 200, 60};     // 绿=可通行
                    case CellFlag::ObstacleUp:  return {220, 50, 40};     // 红=凸起
                    case CellFlag::CliffDown:   return {40, 110, 220};    // 蓝=坑
                    case CellFlag::TooSteep:    return {230, 140, 20};    // 橙=过陡
                    default:                    return {45, 45, 45};      // 深灰=未知
                }
            };
            if (!grid_ok) {
                // 无平面/无网格: 直接刷"深灰=未知"; 此时 hm.flag 为空, 读它会越界
                for (size_t o = 0; o + 2 < img.data.size(); o += 3) {
                    img.data[o] = 45; img.data[o + 1] = 45; img.data[o + 2] = 45;
                }
            } else {
                for (int r = 0; r < hm.rows; ++r) {
                    for (int c = 0; c < hm.cols; ++c) {
                        const CellFlag cf = plane_ok
                            ? hm.flag[static_cast<size_t>(r) * hm.cols + c]
                            : CellFlag::Unknown;   // 无平面 ⇒ 一律画未知 (不假装可通行)
                        const auto col = color_of(cf);
                        for (int dy = 0; dy < sc; ++dy) {
                            for (int dx = 0; dx < sc; ++dx) {
                                const int x = r * sc + dx, y = c * sc + dy;
                                const size_t o = (static_cast<size_t>(y) * W + x) * 3;
                                img.data[o] = col[0]; img.data[o + 1] = col[1]; img.data[o + 2] = col[2];
                            }
                        }
                    }
                }
            }
            tpub->publish(img);
            // ---- v2.9.13 紧凑地形话题 (给上位机/弱网用; 纯新增, 不影响任何判定) ----
            //   背景: /safety/terrain_map 是 x4 放大的 rgb8 渲染图 = 232704 字节/帧
            //         ⇒ 10Hz 需 18.6 Mbit/s; 实测现场 WiFi 只有 ~1.25 Mbit/s ⇒ 上位机订不动。
            //   本话题发**原生网格**(每格 1 字节): 4848 字节/帧 ⇒ 10Hz 仅 0.39 Mbit/s (小 46 倍)。
            //   布局: row-major, index = r*cols + c; cols = x(前)方向(0.6→3.0m, 5cm/格),
            //         rows = y(左右)方向(-2.5→+2.5m); 与内部 hm.flag 同序同义。
            //   取值 = mechdog::CellFlag: 0=Unknown 1=Traversable 2=ObstacleUp 3=CliffDown 4=TooSteep
            //   无平面/无网格时发全 0(=Unknown), 话题节奏恒定 (同 v2.9.8 对 terrain_map 的约定)。
            sensor_msgs::msg::Image gimg;
            gimg.header.stamp = img.header.stamp;
            gimg.header.frame_id = "base_link";
            gimg.height = rows; gimg.width = cols;
            gimg.encoding = "mono8"; gimg.is_bigendian = 0;
            gimg.step = static_cast<sensor_msgs::msg::Image::_step_type>(cols);
            gimg.data.assign(static_cast<size_t>(cols) * static_cast<size_t>(rows), 0);
            if (grid_ok && plane_ok) {
                for (int r = 0; r < hm.rows; ++r) {
                    for (int c = 0; c < hm.cols; ++c) {
                        const size_t i = static_cast<size_t>(r) * hm.cols + c;
                        gimg.data[i] = static_cast<uint8_t>(hm.flag[i]);
                    }
                }
            }
            grpub->publish(gimg);
            std_msgs::msg::String s;
            char buf[512];
            if (plane_ok) {
                std::snprintf(buf, sizeof(buf),
                    "trav=%d  up=%d  down=%d  steep=%d   unknown=%d/4848   in_fov=%d/%d cov=%.0f%%   |  plane tilt=%.1f deg  h0=%.2fm",
                    hm.count_traversable, hm.count_up, hm.count_down, hm.count_steep, hm.count_unknown,
                    hm.count_in_fov, hm.cols * hm.rows, hm.fov_coverage() * 100.0,
                    std::acos(std::min(1.0, std::max(-1.0, static_cast<double>(seg_.plane.nz)))) *
                        180.0 / 3.14159265358979323846,
                    static_cast<double>(seg_.plane.height_at_origin()));
            } else if (depth_gate_hit_) {
                // v2.9.12: 与"真没地面"区分开 —— 这是坏数据被守门拦下, 不是构图问题
                std::snprintf(buf, sizeof(buf),
                    "DEPTH GATE (bad frame: valid=%.1f%% / %zu px) - terrain NOT injected this round | NOT a mounting/aim problem; retry or check camera/USB",
                    depth_gate_vr_ * 100.0, depth_gate_px_);
            } else {
                std::snprintf(buf, sizeof(buf),
                    "NO PLANE (fail-closed) - no ground plane in view; terrain NOT injected this round | in_fov=%d/%d cov=%.0f%% | plane tilt=--  h0=--  (pitch camera down onto open floor)",
                    hm.count_in_fov, cols * rows, hm.fov_coverage() * 100.0);
            }
            s.data = buf;
            if (depth_degraded_) {   // v2.9.17 (口径②) / v2.9.20 (S1): "连续坏帧"+降级链 显式写进状态文本
                char d2[224];
                std::snprintf(d2, sizeof(d2),
                    "  |  DEPTH DEGRADED: 连续 %d 轮无可用深度 (%.1fs) -- 查相机/USB/光照, 非安装角%s",
                    depth_bad_streak_, this->now().seconds() - depth_bad_t0_,
                    degraded_policy_ ? " | 降级链:限速+阈值20-40-70cm" : "");
                s.data += d2;
            }
            spub->publish(s);
        }

        // 诊断 (真机排查): 平面 / 2.5D / 负障碍 状态。
        // 没有这条日志时, "路1 一声不吭"只能靠猜 —— fail-closed 静默是安全设计,
        // 但排查时必须能看到"为什么静默"(无平面? 走廊空? 还是根本没跑到这里)。
        // v2.9.4 安装角自检: 拟合平面的 tilt 就是"安装角参数 vs 实际装配"的误差度量 ——
        //   实测证据 (2026-09-20): 参数按 15°(0.2618) 而镜头实际水平 ⇒ tilt 稳定 13.88°;
        //   参数改按 0.0 ⇒ tilt 0.5~2.7°。持续大 tilt 就是"参数与装配不符", 大声告警。
        //   (这类不一致曾让我们追查数日: 见 E:\33\mechdog_navigation_fixed\PROGRESS_2026-09-19.md §⑧)
        if (seg_.plane.valid) {
            const double tilt_deg = std::acos(std::clamp(seg_.plane.nz, -1.0, 1.0)) * 57.29577951308232;
            if (tilt_deg > 5.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "安装角自检: 拟合平面持续倾斜 %.1f° (>5°) —— cloud_pitch_rad=%.4f rad (%.1f°) 很可能与"
                    "**实际装配**不符; 请核对相机安装角 (装配水平⇒0.0; 装配规格 15°下压⇒0.2618)",
                    tilt_deg, cloud_E_.pitch, cloud_E_.pitch * 57.29577951308232);
            }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "感知: 点云=%zu 平面valid=%d tilt=%.2f° [n=(%.3f,%.3f,%.3f) 分量 %.2f°/%.2f°] h0=%.3fm 内点=%zu neg=%zu fit=%s | 2.5D %s",
            cloud_base_.points.size(), static_cast<int>(seg_.plane.valid),
            std::acos(std::min(1.0, std::max(-1.0, static_cast<double>(seg_.plane.nz)))) *
                180.0 / 3.14159265358979323846,
            static_cast<double>(seg_.plane.nx), static_cast<double>(seg_.plane.ny),
            static_cast<double>(seg_.plane.nz),
            std::asin(std::min(1.0, std::max(-1.0, static_cast<double>(seg_.plane.nx)))) *
                180.0 / 3.14159265358979323846,
            std::asin(std::min(1.0, std::max(-1.0, static_cast<double>(seg_.plane.ny)))) *
                180.0 / 3.14159265358979323846,
            static_cast<double>(seg_.plane.height_at_origin()),
            static_cast<size_t>(seg_.plane.inliers), seg_.negative_points.size(),
            (seg_.used_ransac ? "ransac" : (seg_.used_cell ? "cell" : "none")),   // v2.9.16
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
                "env=%s cliff=%s min_fwd=%.2fm action=%s vel=(%.2f, %.2f) scan=%zu degraded=%s",
                env_to_str(result.environment),
                result.cliff_detected ? "YES" : "no",
                result.min_forward_distance_m,
                action_to_str(result.recommended_action),  // ROS-6 v2.2: 枚举改字符串名
                cmd.linear, cmd.angular,
                scan_n, result.depth_degraded ? "YES" : "no");
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
    bool grid_wedge_only_ = true;   // v2.8.3 视场楔形(仅统计口径: in_fov/覆盖率)
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
    // v2.9.12 可诊断性: 区分"真没地面"与"深度坏帧被守门拦下" —— 此前两者共用同一句
    // "NO PLANE ... (pitch camera down onto open floor)", 现场会误导 (明明对着地面却被告知朝下压)
    bool    depth_gate_hit_  = false;
    // v2.9.17 (口径 ②) + v2.9.20 (S1, N2): 深度可用性"时域"记录 与 降级链开关
    int     depth_bad_streak_n_ = 3;      // 连续 N 轮无可用深度 ⇒ 降级 (参数 depth_bad_streak_n)
    int     depth_bad_streak_   = 0;      // 当前连续坏帧轮数 (好帧清零)
    double  depth_bad_t0_       = 0.0;    // 本段坏帧起始时刻 (秒)
    bool    depth_degraded_     = false;  // 是否已上报"降级"
    bool    depth_ever_good_    = false;  // 是否见过好帧 (v2.9.18 N9: 启动期也计数, 仅门槛提高)
    int     depth_last_reason_  = -1;     // 最近一次原因: 0=无帧 1=质量守门 2=点云为空
    double  last_degraded_status_s_ = -1e9;  // 上次主动发降级文案的时刻 (1Hz 节流)
    // v2.9.20 (S1, N2): 降级链开关 (参数 degraded_policy; true = 降级期"前进限速≤SLOW + 反应线 20/40/70cm")
    bool    degraded_policy_    = true;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_text_pub_;   // v2.9.18: 构造期已创建
    double  depth_gate_vr_   = 0.0;   // 本轮的深度有效率
    size_t  depth_gate_px_   = 0;     // 本轮的有效像素数
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
    bool        ultra_ever_rx_ = false;        // v2.9.19 (B4): 启动以来收到过首帧
    bool        topic_wait_warned_ = false;    // v2.9.19 (B4): "5s 无首帧"告警只发一次
    std::chrono::steady_clock::time_point last_ultra_rx_{};

    // 深度来源 (v2.4): depth_source=topic 时的订阅与新鲜度状态
    std::string depth_source_ = "auto";
    std::string depth_topic_ = "/camera/depth/image_raw";
    std::string depth_info_topic_ = "/camera/depth/camera_info";
    // v2.9.15: 汇报用小图发布 (1/2 抽点深度; 见 publish_depth_small)
    bool publish_depth_small_ = true;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_small_pub_;
    int  depth_timeout_ms_ = 500;
    bool have_depth_rx_ = false;       // 收到过话题帧 (供超时看门狗判断)
    bool have_camera_info_ = false;
    std::chrono::steady_clock::time_point last_depth_rx_{};
    // v2.9.11 分段计时(仅观测用; 跨线程读取, 允许无锁 — 只用于日志)
    double ms_decode_ = 0.0, ms_inject_ = 0.0;      // 主线程
    double ms_gate_ = 0.0, ms_backproj_ = 0.0, ms_to_base_ = 0.0,
           ms_seg_ = 0.0, ms_heightmap_ = 0.0;      // 感知线程

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
            << ",\"degraded\":" << (r.depth_degraded ? "true" : "false")
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
