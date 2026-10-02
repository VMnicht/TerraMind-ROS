# 实机入口：控制管理 + 串口后端 + 可选面板；不启动仿真或机器人模型。
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('terramind_bringup'))
    serial = share/'config/serial.yaml'
    serial_parameters = yaml.safe_load(serial.read_text())['mcu_serial_node']['ros__parameters']
    # 先读取 YAML 字典再覆盖 device，确保命令行手动端口优先于 device:auto。
    default_device = serial_parameters['device']
    common = str(share/'config/control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value=default_device,
                              description='auto to discover the control board, or an explicit serial path'),
        DeclareLaunchArgument('panel', default_value='false'),
        Node(package='terramind_control', executable='control_manager_node', parameters=[common], output='screen'),
        Node(package='terramind_mcu', executable='mcu_serial_node', output='screen',
             parameters=[common, {**serial_parameters, 'device': LaunchConfiguration('device'), 'simulated': False}]),
        Node(package='terramind_panel', executable='control_panel', parameters=[common],
             condition=IfCondition(LaunchConfiguration('panel')), output='screen'),
    ])
