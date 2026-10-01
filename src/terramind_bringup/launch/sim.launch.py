from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('terramind_bringup'))
    description = Path(get_package_share_directory('terramind_description'))
    common = str(share/'config/control.yaml')
    simulation = share/'config/sim.yaml'
    geometry = yaml.safe_load(simulation.read_text())['sim_interface_node']['ros__parameters']
    radius = float(geometry['wheel_diameter_m'])/2
    half_track = float(geometry['wheel_separation_m'])/2
    if radius <= 0 or half_track <= 0:
        raise ValueError('positive simulation dimensions required')
    model = (description/'urdf/terramind.urdf.in').read_text()
    model = model.replace('@WHEEL_RADIUS@', str(radius)).replace('@HALF_TRACK@', str(half_track)).replace('@NEG_HALF_TRACK@', str(-half_track))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('description', default_value='true'),
        DeclareLaunchArgument('panel', default_value='false'),
        DeclareLaunchArgument('capabilities', default_value=str(geometry['capabilities']),
                              description='Simulation capability mask: 47=current board with sprayer, 15=legacy, 63=all actuators'),
        Node(package='terramind_control', executable='control_manager_node', parameters=[common], output='screen'),
        Node(package='terramind_sim', executable='sim_interface_node',
             # Merge before emitting the parameter file: a node-specific YAML
             # entry can otherwise override the generated wildcard arguments.
             parameters=[common, {**geometry, 'capabilities': ParameterValue(LaunchConfiguration('capabilities'), value_type=int)}], output='screen'),
        Node(package='terramind_panel', executable='control_panel', parameters=[common, {'backend_label': '二维运动仿真'}],
             condition=IfCondition(LaunchConfiguration('panel')), output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model, 'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('description'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(description/'rviz/terramind.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
