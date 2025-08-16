# camera_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义要启动的节点
    camera_pub = Node(
        package='camera',  # 指定包名
        executable='camera',  # 指定可执行文件名
        output='screen',  # 输出日志到屏幕
        respawn=True,  # 让节点在崩溃时自动重启
        respawn_delay=2.0  # 设置重启延迟（可选，单位：秒）
    )

    # 返回LaunchDescription对象，包含所有需要启动的节点
    return LaunchDescription([
        camera_pub,
    ])
