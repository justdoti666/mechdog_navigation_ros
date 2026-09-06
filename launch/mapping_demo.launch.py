"""mapping_demo.launch.py — 建图端到端演示一键启动 (P4 demo)

用法:
    ros2 launch mechdog_navigation_ros mapping_demo.launch.py

    可选参数:
        use_simulated_depth:=false   # 预留: 真机 Astra 深度 (当前实现仅合成帧)
        wall_dist:=3.0               # 合成墙距离 (m)
        frame_period:=0.5            # 深度帧周期 (s)

拉起内容:
    1. quadruped_base/base_odom_dry_run.launch.py (师兄 dry_run 里程计, 位姿归零)
       —— 依赖师兄包已构建于同一工作区 (~/mechdog_ws)
    2. mapping_demo_node (本包: 订阅 /odom_dry_run 位姿 + 合成深度 → /map + PGM)

驱动 "行驶" 需另发速度指令 (launch 不代发, 避免脚本退出后机器人仍在"动"):
    ros2 topic pub -r 10 /odom_test_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

查看结果:
    ros2 topic echo /map --once | head -20
    PGM: ~/mechdog_map.pgm
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wall_dist = LaunchConfiguration('wall_dist')
    frame_period = LaunchConfiguration('frame_period')
    pgm_path = LaunchConfiguration('pgm_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'wall_dist', default_value='3.0',
            description='合成墙距离 (m)'),
        DeclareLaunchArgument(
            'frame_period', default_value='0.5',
            description='深度帧周期 (s)'),
        DeclareLaunchArgument(
            'pgm_path', default_value='/tmp/mechdog_map.pgm',
            description='PGM 地图落盘路径'),

        # --- 师兄 dry_run 里程计 (同工作区 quadruped_base 包) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('quadruped_base'), 'launch',
                'base_odom_dry_run.launch.py',
            ])),
        ),

        # --- 本包建图演示节点 ---
        Node(
            package='mechdog_navigation_ros',
            executable='mapping_demo_node',
            name='mapping_demo_node',
            output='screen',
            parameters=[{
                'wall_dist_m': wall_dist,
                'frame_period_sec': frame_period,
                'pgm_path': pgm_path,
            }],
        ),
    ])
