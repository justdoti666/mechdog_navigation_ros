"""safety_node + chassis_bridge_node 启动文件

用法:
    ros2 launch mechdog_navigation_ros safety.launch.py
    ros2 launch mechdog_navigation_ros safety.launch.py use_simulated:=false   # 真机传感器
    ros2 launch mechdog_navigation_ros safety.launch.py bridge_type:=stm32     # 真机底盘
    ros2 launch mechdog_navigation_ros safety.launch.py cmd_vel_topic:=/cmd_vel  # 绕过安全闸门, 仅测试

链路说明 (H2):
    - 默认 safety_node 发 /unsafe/cmd_vel, chassis_bridge_node 订阅 /cmd_vel ——
      两者在**孤立 launch** 下互不通信, 中间必须由师兄 quadruped_ws 的
      cmd_vel_safety_gate_node 桥接 (订阅 /unsafe/cmd_vel, 经安全检查转发 /cmd_vel)。
    - 若未运行 quadruped_ws (PC 模拟验证), 请加 cmd_vel_topic:=/cmd_vel 让
      safety_node 直连 /cmd_vel —— 用于验证融合/规划/桥接全链路, 但**绕过安全闸门**,
      仅限测试, 不要用于真机联调。
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_simulated', default_value='true',
            description='true=PC 模拟模式(无需硬件), false=真机模式'),
        DeclareLaunchArgument(
            'bridge_type', default_value='simulated',
            description='simulated=模拟底盘(默认), stm32=STM32 真机底盘(串口 21 字节帧, 已实现)'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/unsafe/cmd_vel',
            description='速度指令发布话题 (默认 /unsafe/cmd_vel 走师兄安全闸门; '
                        '测试直连链路可设 /cmd_vel, 绕过闸门仅限测试)'),
        DeclareLaunchArgument(
            'enable_pointcloud', default_value='false',
            description='近场点云: safety_node 发布深度点云到 /mechdog/point_cloud '
                        '(camera_link 系, 供 Nav2 voxel_layer 标记悬空/立体障碍)'),
        DeclareLaunchArgument(
            'camera_x', default_value='0.12',
            description='相机相对 base_link: 前 (m, 外参占位值, 装机后量测)'),
        DeclareLaunchArgument(
            'camera_z', default_value='0.18',
            description='相机相对 base_link: 高 (m)'),
        DeclareLaunchArgument(
            # ⚠ 跟随**实际装配**: 台架实测≈10°前倾 ⇒ 0.17 (见 PLAN_2026-09-21 §P1);
            #   机械按规格装成 15° 下压后改 0.2618。改动前务必用"安装角自检"确认 tilt<2°。
            'camera_pitch_rad',
            default_value='0.0',
            description='相机俯仰 (rad, +15° 前俯, 与算法库 CameraExtrinsics 默认一致)'),
        DeclareLaunchArgument(
            'enable_rgb', default_value='false',
            description='RGB 回传: safety_node 发布 Astra 彩色帧到 /mechdog/rgb/image_raw '
                        '(替代支架相机/USB 相机, 供温度-视觉验证与 Foxglove 回传; 真机出图)'),
        DeclareLaunchArgument(
            'depth_source', default_value='auto',
            description='深度来源: auto=sdk(编译了 SDK)/topic(未编译, 如 Pi) | '
                        'topic=订阅 ROS 深度话题 (无需 Astra SDK) | sdk=Astra SDK 直读 | '
                        'simulated=模拟帧'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth/image_raw',
            description='depth_source=topic 时的深度话题 (16UC1/mono16/32FC1)'),
        DeclareLaunchArgument(
            'depth_timeout_ms', default_value='500',
            description='深度话题超时 (ms): 超过则标记深度帧失效 (fail-closed, 决策只剩超声)'),
        DeclareLaunchArgument(
            'ultrasonic_source', default_value='auto',
            description='超声来源: auto(有/ultrasonic发布者→topic; GPIO就绪→hardware; 否则→none) | '
                        'topic | hardware | simulated(仅台架) | none。'
                        '真机上**绝不**静默使用模拟随机数 (底部 5% 概率造悬崖 → 假 STOP)'),
        DeclareLaunchArgument(
            'allow_simulated_ultrasonic', default_value='false',
            description='台架调试用: 允许真实模式下把模拟超声接进安全链 (默认拒绝)'),
        DeclareLaunchArgument(
            'ultrasonic_timeout_ms', default_value='500',
            description='超声话题超时 (ms): 超过则把超声移出安全链 (避免回落模拟随机数)'),
        DeclareLaunchArgument(
            'camera_height_m', default_value='-1.0',
            description='相机镜头离**地面**高度(m): >0 时推导地面高度先验 = -(h - camera_z)。'
                        '台架(相机架高约 0.6m)用 camera_height_m:=0.6 camera_z:=0；'
                        '装机后改回 0.2；-1 = 用仓库装机默认(-0.18)'),
        DeclareLaunchArgument(
            'ground_prior_z', default_value='-999.0',
            description='直接指定地面高度先验 (优先于 camera_height_m 推导); -999 = 不指定'),
        DeclareLaunchArgument(
            'ground_prior_window', default_value='-1.0',
            description='地面高度先验半带宽(m); <=0 = 用仓库默认(0.10)'),
        DeclareLaunchArgument(
            'camera_roll_rad', default_value='0.0',
            description='v2.8 相机安装横滚(弧度); 竖墙标定给出 (台架实测 -0.041)'),
        DeclareLaunchArgument(
            'camera_yaw_rad', default_value='0.0',   # v2.9.6 安装偏航 (单水平面观测不出, 装夹对齐机体后置 0)
            description='v2.9.6 相机安装偏航(弧度); 单水平面观测不出, 装夹对齐机体后置 0'),
        DeclareLaunchArgument(
            'ground_fit_method', default_value='ransac',
            description='v2.7 地面提取方法: ransac(默认, 与历史一致) | cell(确定性格最小拟合)'),
        DeclareLaunchArgument(
            'cell_skip_ransac', default_value='true',
            description='v2.9.16 地面提取: cell 成功时跳过 RANSAC (口径①; false=回历史行为"cell 后仍被 RANSAC 覆盖")'),
        DeclareLaunchArgument(
            'depth_bad_streak_n', default_value='3',
            description='v2.9.17 深度守门时域: 连续 N 轮拿不到可用深度 ⇒ 降级并向 /safety/status_text 显式上报'),
        DeclareLaunchArgument(
            'publish_depth_small', default_value='true',
            description='v2.9.15 发布 /safety/depth_small (16UC1 320x240, 供汇报窗口; 关掉可省带宽)'),
        Node(
            package='mechdog_navigation_ros',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[{
                'use_simulated': LaunchConfiguration('use_simulated'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
                'enable_rgb': LaunchConfiguration('enable_rgb'),
                'depth_source': LaunchConfiguration('depth_source'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'depth_timeout_ms': LaunchConfiguration('depth_timeout_ms'),
                'ultrasonic_source': LaunchConfiguration('ultrasonic_source'),
                'allow_simulated_ultrasonic': LaunchConfiguration('allow_simulated_ultrasonic'),
                'ultrasonic_timeout_ms': LaunchConfiguration('ultrasonic_timeout_ms'),
                # v2.6: 算法侧外参与静态 TF **同源** (修掉"TF 用 launch 值/算法用硬编码")
                'cloud_x': LaunchConfiguration('camera_x'),
                'cloud_z': LaunchConfiguration('camera_z'),
                'cloud_pitch_rad': LaunchConfiguration('camera_pitch_rad'),
                'cloud_yaw_rad': LaunchConfiguration('camera_yaw_rad'),
                'cloud_roll_rad': LaunchConfiguration('camera_roll_rad'),
                'camera_height_m': LaunchConfiguration('camera_height_m'),
                'ground_prior_z': LaunchConfiguration('ground_prior_z'),
                'ground_prior_window': LaunchConfiguration('ground_prior_window'),
                'ground_fit_method': LaunchConfiguration('ground_fit_method'),
                'cell_skip_ransac': LaunchConfiguration('cell_skip_ransac'),
                'depth_bad_streak_n': LaunchConfiguration('depth_bad_streak_n'),
                'publish_depth_small': LaunchConfiguration('publish_depth_small'),
            }],
        ),
        # 近场点云坐标: base_link -> camera_link 静态变换 (roll/pitch/yaw 弧度;
        # 与算法库 CameraExtrinsics 默认值一致, 外参标定后同步更新).
        # 注意: 必须用键值对形式 —— Iron 起 static_transform_publisher 位置参数已弃用,
        # lyrical 上位置参数直接解析失败 (Frame id must not be empty), 真机实测踩过.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_pointcloud')),
            arguments=[
                '--x', LaunchConfiguration('camera_x'),
                '--y', '0.0',
                '--z', LaunchConfiguration('camera_z'),
                '--roll', '0.0',
                '--pitch', LaunchConfiguration('camera_pitch_rad'),
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),
        Node(
            package='mechdog_navigation_ros',
            executable='chassis_bridge_node',
            name='chassis_bridge_node',
            output='screen',
            parameters=[{
                'bridge_type': LaunchConfiguration('bridge_type'),
            }],
        ),
    ])
