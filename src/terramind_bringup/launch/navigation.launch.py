"""独立导航接收入口；不启动控制后端，不发送任何串口字节。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def start_navigation(context):
    share = Path(get_package_share_directory('terramind_bringup'))
    config = Path(LaunchConfiguration('config').perform(context))
    parameters = yaml.safe_load(config.read_text())['navigation_serial_node']['ros__parameters']
    device = LaunchConfiguration('device').perform(context).strip()
    if device:
        parameters['device'] = device
    nodes = [Node(package='terramind_navigation_driver', executable='navigation_serial_node',
                  parameters=[parameters], output='screen')]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(share/'config/navigation.rviz'),
                                     '-f', parameters.get('map_frame', 'navigation_map')]))
    return nodes


def generate_launch_description():
    config = Path(get_package_share_directory('terramind_bringup')) / 'config/navigation.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('device', default_value='', description='Empty uses device from config'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=start_navigation),
    ])
