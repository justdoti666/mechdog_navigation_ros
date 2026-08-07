"""safety_node + chassis_bridge_node 启动文件

用法:
    ros2 launch mechdog_navigation_ros safety.launch.py
    ros2 launch mechdog_navigation_ros safety.launch.py use_simulated:=false   # 真机传感器
    ros2 launch mechdog_navigation_ros safety.launch.py bridge_type:=stm32     # 真机底盘
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
            description='simulated=模拟底盘(默认), stm32=STM32 真机底盘(待硬件确认后实现)'),
        Node(
            package='mechdog_navigation_ros',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[{
                'use_simulated': LaunchConfiguration('use_simulated'),
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
