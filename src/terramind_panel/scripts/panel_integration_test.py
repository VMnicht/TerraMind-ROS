#!/usr/bin/env python3
"""Exercise the actual Qt controls against isolated simulation/PTY processes."""
import argparse
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['sim', 'serial'], required=True)
    parser.add_argument('--capabilities', type=int, choices=[15, 63], default=63)
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(40, 100) if args.mode == 'sim' else random.randint(110, 200))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    from ament_index_python.packages import get_package_prefix
    from PyQt5 import QtWidgets
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import SetBool
    from terramind_panel.controls import ACTUATORS
    from terramind_panel.controller import PanelNode
    from terramind_panel.window import PanelWindow

    processes = []
    with tempfile.TemporaryDirectory(prefix='terramind-panel-test-') as directory:
        os.environ['ROS_LOG_DIR'] = str(Path(directory) / 'ros_logs')
        log = open(Path(directory) / 'children.log', 'w+')

        def launch(package, executable, *arguments):
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            process = subprocess.Popen([str(binary), *arguments], stdout=log, stderr=log)
            processes.append(process)
            return process

        rclpy.init()
        app = QtWidgets.QApplication([])
        app.setStyle('Fusion')
        node = PanelNode()
        window = PanelWindow(node)
        window.show()

        def wait(predicate, timeout=6):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                app.processEvents()
                if not window.timer.isActive():
                    for _ in range(8):
                        rclpy.spin_once(node, timeout_sec=0)
                    node.tick()
                if predicate():
                    return
                time.sleep(.002)
            raise AssertionError(f'{args.mode}: {node.phase}; {node.reason}; {node.health_error()}; '
                                 f'control={node.control}; events={list(node.events)}')

        def pump(duration):
            end = time.monotonic() + duration
            wait(lambda: time.monotonic() >= end, timeout=duration + 1)

        def start():
            wait(lambda: not node.health_error() and node.enable_future is None and not node.control.enabled)
            assert node.board.simulated, 'refusing to enable a backend not marked simulated'
            window.refresh()
            assert window.start_button.isEnabled()
            window.start_button.click()
            wait(lambda: node.phase == 'running' and node.board.mode == 1 and node.board.mower.on)

        def stopped():
            return (node.phase == 'idle' and node.board.mode == 2 and not node.control.enabled
                    and node.board.chassis.left_target_rpm == 0 and not node.board.mower.on)

        try:
            link = str(Path(directory) / 'mcu')
            emulator = None
            if args.mode == 'serial':
                emulator = launch('terramind_sim', 'serial_board_emulator', '--link', link,
                                  '--fragment', '3', '--capabilities', str(args.capabilities))
                launch('terramind_mcu', 'mcu_serial_node', '--ros-args', '-p', f'device:={link}',
                       '-p', 'simulated:=true', '-p', 'reconnect_interval_s:=0.2')
            else:
                launch('terramind_sim', 'sim_interface_node', '--ros-args', '-p', f'capabilities:={args.capabilities}')
            manager = launch('terramind_control', 'control_manager_node')
            wait(lambda: not node.health_error() and not node.input_conflict()
                 and node.velocity.get_subscription_count() and node.implements.get_subscription_count()
                 and node.enable_client.service_is_ready())
            assert stopped()
            window.refresh()
            assert window.checks['lift'].isEnabled() == bool(args.capabilities & 16)
            assert window.checks['sprayer'].isEnabled() == bool(args.capabilities & 32)
            window.editors['linear_mps'].setValue(.12)
            window.editors['angular_radps'].setValue(-.4)
            targets = dict(left_rpm=-120., right_rpm=180., mower_percent=12., lift_mm=80., sprayer_percent=25.)
            for field in ACTUATORS:
                window.editors[field.value_field].setValue(targets[field.value_field])
                window.checks[field.key].setChecked(bool(args.capabilities & (1 << field.bit)))
            start()
            wait(lambda: abs(node.board.chassis.linear_mps - .12) < 1e-5
                 and abs(node.board.chassis.angular_radps + .4) < 1e-5
                 and node.board.left_spreader.target_rpm == -120.
                 and node.board.right_spreader.target_rpm == 180.
                 and node.board.mower.throttle_set_percent == 12.
                 and (not args.capabilities & 16 or node.board.lift.target_mm == 80.)
                 and (not args.capabilities & 32 or node.board.sprayer.target_percent == 25.))
            pump(.3)
            assert node.board.result == 0
            if args.screenshot:
                window.refresh()
                assert window.grab().save(str(args.screenshot.resolve()))

            # Editing a draft must not change the transmitted snapshot before Apply.
            old_sequence = node.board.last_command_seq
            window.editors['linear_mps'].setValue(-.08)
            window.editors['left_rpm'].setValue(90.)
            pump(.15)
            assert abs(node.board.chassis.linear_mps - .12) < 1e-5
            assert node.board.left_spreader.target_rpm == -120.
            window.apply_button.click()
            wait(lambda: abs(node.board.chassis.linear_mps + .08) < 1e-5
                 and node.board.left_spreader.target_rpm == 90.)
            assert node.board.last_command_seq != old_sequence
            # Off blocks must transmit zero, while preserving their editable target.
            window.checks['right'].setChecked(False)
            window.apply_button.click()
            wait(lambda: not node.board.right_spreader.on and node.board.right_spreader.target_rpm == 0)
            assert window.editors['right_rpm'].value() == 180.
            window.stop_button.click()
            wait(stopped)
            pump(.25)
            assert stopped()
            start()
            window.disable_button.click()
            wait(stopped)

            # Deliberately freeze Qt; no publishing worker may retain old targets.
            start()
            time.sleep(.35)
            wait(stopped)
            pump(.2)
            assert stopped()

            # Loss/replacement of the backend requires explicit operator restart.
            start()
            if args.mode == 'sim':
                client = node.create_client(SetBool, 'sim/drop_status')
                wait(client.service_is_ready)
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)
                wait(future.done)
                wait(lambda: node.phase == 'idle')
                request.data = False
                future = client.call_async(request)
                wait(future.done)
                wait(stopped)
            else:
                old_id = node.board.connection_id
                emulator.terminate()
                emulator.wait(timeout=3)
                wait(lambda: node.phase == 'idle')
                emulator = launch('terramind_sim', 'serial_board_emulator', '--link', link,
                                  '--capabilities', str(args.capabilities))
                wait(lambda: not node.health_error() and node.board.connection_id != old_id)
                wait(stopped)

            # Stop during the initial Stop barrier, then during the Enable stage.
            window.refresh()
            window.start_button.click()
            window.stop_button.click()
            wait(lambda: node.enable_future is None and stopped())
            pump(.3)
            assert stopped()
            window.refresh()
            window.start_button.click()
            wait(lambda: node.enable_stage == 'enable')
            window.stop_button.click()
            wait(lambda: node.enable_future is None and stopped())
            pump(.3)
            assert stopped()

            # An unresolved service response must expire locally and allow recovery.
            if args.mode == 'sim':
                window.refresh()
                manager.send_signal(signal.SIGSTOP)
                try:
                    window.start_button.click()
                    assert node.enable_future is not None
                    wait(lambda: node.phase == 'idle' and node.enable_future is None, timeout=4)
                finally:
                    manager.send_signal(signal.SIGCONT)
                wait(lambda: not node.health_error() and stopped())
                pump(.3)
                assert stopped()

            # Merely connecting a second input publisher must block panel ownership.
            start()
            conflict = node.create_publisher(TwistStamped, 'cmd_vel', 1)
            wait(stopped)
            try:
                node.start(window.draft())
                raise AssertionError('second control input was not rejected')
            except ValueError as error:
                assert '其他控制输入' in str(error)
            node.destroy_publisher(conflict)
            wait(lambda: not node.input_conflict())

            start()
            window.close()
            wait(lambda: not window.isVisible() and stopped())
            print(f'PASS panel {args.mode} caps={args.capabilities}: all fields, draft/apply, off-zero, stop/disable, Qt stall, link recovery, enable/stop race, input conflict, close')
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            window.timer.stop()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for process in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in reversed(processes):
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.close()


if __name__ == '__main__':
    main()
