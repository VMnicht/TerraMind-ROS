"""独立导航接收入口；不启动控制后端，不发送任何串口字节。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    config = Path(get_package_share_directory('terramind_bringup')) / 'config/navigation.yaml'
    parameters = yaml.safe_load(config.read_text())['navigation_serial_node']['ros__parameters']
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value=parameters['device']),
        Node(package='terramind_navigation_driver', executable='navigation_serial_node',
             parameters=[{**parameters, 'device': LaunchConfiguration('device')}], output='screen'),
    ])
