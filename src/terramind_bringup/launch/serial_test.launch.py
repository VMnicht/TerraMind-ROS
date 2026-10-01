from pathlib import Path
import shutil
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def setup(context):
    capabilities = int(LaunchConfiguration('capabilities').perform(context))
    if not 0 <= capabilities <= 63:
        raise ValueError('capabilities must be in [0, 63]')
    share = Path(get_package_share_directory('terramind_bringup'))
    common = str(share/'config/control.yaml')
    geometry = yaml.safe_load((share/'config/sim.yaml').read_text())['sim_interface_node']['ros__parameters']
    directory = tempfile.mkdtemp(prefix='terramind-serial-')
    link = str(Path(directory)/'mcu')
    emulator = str(Path(get_package_prefix('terramind_sim'))/'lib/terramind_sim/serial_board_emulator')

    def cleanup(event, context):
        # Only remove the private directory allocated by this launch instance.
        shutil.rmtree(directory, ignore_errors=True)
        return []

    return [
        ExecuteProcess(cmd=[emulator, '--link', link, '--wheel-diameter', str(geometry['wheel_diameter_m']),
                            '--wheel-separation', str(geometry['wheel_separation_m']),
                            '--capabilities', str(capabilities)], output='screen'),
        Node(package='terramind_control', executable='control_manager_node', parameters=[common], output='screen'),
        Node(package='terramind_mcu', executable='mcu_serial_node', output='screen',
             parameters=[common, {'device': link, 'simulated': True, 'reconnect_interval_s': 0.2}]),
        Node(package='terramind_panel', executable='control_panel',
             parameters=[common, {'backend_label': '虚拟串口 · ' + link}],
             condition=IfCondition(LaunchConfiguration('panel')), output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('panel', default_value='false'),
        DeclareLaunchArgument('capabilities', default_value='15',
                              description='Emulated capability mask: 15=current board, 63=all actuators'),
        OpaqueFunction(function=setup),
    ])
