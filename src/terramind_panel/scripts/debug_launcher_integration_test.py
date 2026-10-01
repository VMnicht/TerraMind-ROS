#!/usr/bin/env python3
"""Exercise launcher buttons with real ROS processes and simulated hardware only."""
import argparse
import os
from pathlib import Path
import random
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
    from terramind_interfaces.msg import McuState, ControlStatus

    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    rclpy.init()
    node = rclpy.create_node('debug_launcher_integration_test')
    board, control, diagnostics = [], [], []
    node.create_subscription(McuState, 'mcu/state', board.append, 1)
    node.create_subscription(ControlStatus, 'control/status', control.append, 1)
    node.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    window = DebugWindow(workspace)
    window.show()
    window.rviz.setChecked(False)

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
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

        # A deliberately nonexistent explicit path must override device:auto.
        # This proves the real launch does not scan or open any actual devices.
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
