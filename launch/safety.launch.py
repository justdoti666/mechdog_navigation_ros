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
        Node(
            package='mechdog_navigation_ros',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[{
                'use_simulated': LaunchConfiguration('use_simulated'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
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
