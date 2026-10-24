#!/usr/bin/env python3
"""操作总面板按钮验证真实 ROS 子进程；只使用模拟硬件和不存在的实机路径。"""
import argparse
import os
from pathlib import Path
import random
import binascii
import math
import signal
import struct
import subprocess
import tempfile
import time


def main():
    from terramind_panel.debug_panel import DebugWindow, find_workspace
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=find_workspace())
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    if args.workspace is None:
        parser.error('workspace not found; specify --workspace')
    workspace = args.workspace.resolve()
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(40, 200))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    os.environ['ROS_LOG_DIR'] = str(workspace / 'log/debug-integration')
    from PyQt5 import QtWidgets
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from terramind_interfaces.msg import McuState, ControlStatus, NavigationStatus
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix
    import yaml

    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    rclpy.init()
    node = rclpy.create_node('debug_launcher_integration_test')
    board, control, diagnostics = [], [], []
    navigation, poses = [], []
    node.create_subscription(NavigationStatus, 'navigation/status', navigation.append, 1)
    node.create_subscription(PoseStamped, 'navigation/pose', poses.append, qos_profile_sensor_data)
    node.create_subscription(McuState, 'mcu/state', board.append, 1)
    node.create_subscription(ControlStatus, 'control/status', control.append, 1)
    node.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    window = DebugWindow(workspace)
    window.show()
    window.rviz.setChecked(False)

    def wait(predicate, timeout=15, writer=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if writer:
                writer()
            app.processEvents()
            rclpy.spin_once(node, timeout_sec=.005)
            if predicate():
                return
        raise AssertionError('\n'.join(job.buffer for job in window.jobs)[-16000:])

    try:
        for index in (0, 1):
            board.clear()
            control.clear()
            window.mode.setCurrentIndex(index)
            window.capabilities.setCurrentIndex(0)
            window.launch_button.click()
            wait(lambda: board and control and board[-1].link_ready and control[-1].link_ready
                 and node.count_publishers('cmd_vel') == 1)
            assert board[-1].simulated and board[-1].capabilities == 47 and not control[-1].enabled
            assert not window.launch_button.isEnabled() and not window.panel_button.isEnabled()
            if index == 0:
                if args.screenshot:
                    window.grab().save(str(args.screenshot))
                window.observe_button.click()
                wait(lambda: any('connection_id:' in job.buffer for job in window.jobs
                                 if job.task.key.startswith('topic-')))
            window.stop_all_button.click()
            wait(lambda: not window.active(), timeout=20)
            assert all(job.state == '已停止' for job in window.jobs)
            print('PASS launch, ready, panel, stop:', index, flush=True)

        # 故意指定不存在的设备路径，验证它覆盖 device:auto。
        # 该分支不得扫描或打开任何实际控制板。
        device = f'/dev/terramind-launcher-test-absent-{os.getpid()}'
        assert not Path(device).exists()
        window.mode.setCurrentIndex(2)
        window.device.setText(device)
        window.panel.setChecked(False)
        diagnostics.clear()
        window.launch_button.click()
        wait(lambda: any(status.name == 'mcu_serial' and status.hardware_id == device
                         for message in diagnostics for status in message.status))
        window.stop_all_button.click()
        wait(lambda: not window.active(), timeout=20)
        print('PASS explicit device overrides auto (nonexistent path)', flush=True)

        # 两个独立私有串口，经过真实 real.launch + 自定义配置启动全链路。
        with tempfile.TemporaryDirectory(prefix='terramind-joint-') as directory:
            directory = Path(directory)
            master, slave = os.openpty()
            emulator = None
            try:
                link = directory / 'control'
                executable = Path(get_package_prefix('terramind_sim')) / 'lib/terramind_sim/serial_board_emulator'
                emulator = subprocess.Popen([str(executable), '--link', str(link), '--capabilities', '63'],
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                wait(link.exists)
                config = directory / 'navigation.yaml'
                config.write_text(yaml.safe_dump({'navigation_serial_node': {'ros__parameters': {
                    'device': '/dev/terramind-should-be-overridden', 'allow_simulated': True,
                    'origin_mode': 'fixed', 'origin_latitude_deg': math.degrees(.5),
                    'origin_longitude_deg': math.degrees(2.), 'origin_height_m': 20.,
                    'extrinsics_calibrated': True, 'imu_translation_in_base_m': [1., 0., 0.],
                    'map_frame': 'test_field',
                }}}))
                window.device.setText(str(link))
                # PTY 链接在 /tmp，不通过面板的 /dev/ 校验；使用其真实 /dev/pts 路径。
                window.device.setText(str(link.resolve()))
                window.navigation.setChecked(True)
                window.navigation_device.setText(os.ttyname(slave))
                window.navigation_config.setText(str(config))
                window.panel.setChecked(True)
                board.clear(); navigation.clear(); poses.clear()
                window.launch_button.click()
                wait(lambda: navigation and board and board[-1].link_ready)
                sequence = [0]

                def feed_navigation():
                    sequence[0] += 1
                    q = math.sqrt(.5)
                    packet = bytearray(struct.pack('<2sBBIdddf3f3f4fHH', b'\xaa\x55', 1, 0x9D,
                        sequence[0], 1000. + sequence[0] * .01, .5, 2., 20.,
                        0., 1., 0., 0., 0., math.pi/2, q, 0., 0., q, 200, 0))
                    struct.pack_into('<H', packet, 78, binascii.crc_hqx(packet[2:78], 0xFFFF))
                    os.write(master, packet)

                wait(lambda: navigation[-1].usable and poses, writer=feed_navigation)
                assert board[-1].capabilities == 63
                assert navigation[-1].origin_ready and navigation[-1].extrinsics_calibrated
                assert navigation[-1].pose_child_frame == 'base_link'
                assert poses[-1].header.frame_id == 'test_field'
                assert abs(poses[-1].pose.position.x + 1) < 1e-5
                wait(lambda: not navigation[-1].usable)
                wait(lambda: navigation[-1].usable, writer=feed_navigation)
                print('PASS joint control/navigation launch, config, extrinsics and stream recovery', flush=True)
            finally:
                window.stop_all()
                wait(lambda: not window.active(), timeout=20)
                if emulator is not None:
                    emulator.send_signal(signal.SIGINT)
                    try:
                        emulator.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        emulator.kill(); emulator.wait()
                os.close(master); os.close(slave)

        window.test_choice.setCurrentIndex(1)
        window.test_button.click()
        wait(lambda: not window.active(), timeout=30)
        assert window.jobs[-1].state == '已完成', window.jobs[-1].buffer
        print('PASS test from UI', flush=True)
    finally:
        window.stop_all()
        wait(lambda: not window.active(), timeout=20)
        window.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
