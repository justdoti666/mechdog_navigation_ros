"""safety_node 启动文件

用法:
    ros2 launch mechdog_navigation_ros safety.launch.py
    ros2 launch mechdog_navigation_ros safety.launch.py use_simulated:=false   # 真机
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
        Node(
            package='mechdog_navigation_ros',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[{
                'use_simulated': LaunchConfiguration('use_simulated'),
            }],
        ),
    ])
