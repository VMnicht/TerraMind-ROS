#!/usr/bin/env python3
"""Automatic discovery integration test using ONLY private PTYs and a private ROS domain."""
import errno
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time
import tty


def main():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(110, 200))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    from ament_index_python.packages import get_package_prefix
    import rclpy
    import yaml
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import SetBool
    from terramind_interfaces.msg import McuState, ControlStatus

    processes = []
    with tempfile.TemporaryDirectory(prefix='terramind-autoserial-') as directory:
        root = Path(directory)
        ports = root / 'ports'
        preferred = root / 'by-id'
        ports.mkdir()
        preferred.mkdir()
        os.environ['ROS_LOG_DIR'] = str(root / 'logs')
        config = root / 'driver.yaml'
        config.write_text(yaml.safe_dump({'mcu_serial_node': {'ros__parameters': {
            'device': 'auto', 'simulated': True, 'discovery_listen_s': .3,
            'reconnect_interval_s': .2,
            'discovery_patterns': [str(preferred / '*'), str(ports / '*')],
        }}}))
        log = open(root / 'children.log', 'w+')
        peers = []
        received_on_navigation = bytearray()
        next_navigation = 0.

        def launch(package, executable, *arguments):
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            process = subprocess.Popen([str(binary), *arguments], stdout=log, stderr=log)
            processes.append(process)
            return process

        def driver():
            return launch('terramind_mcu', 'mcu_serial_node', '--ros-args', '--params-file', str(config))

        def emulator(path):
            process = launch('terramind_sim', 'serial_board_emulator', '--link', str(path), '--fragment', '3')
            wait(path.exists)
            return process

        def terminate(process):
            process.send_signal(signal.SIGINT)
            process.wait(timeout=3)

        rclpy.init()
        node = Node('serial_discovery_test')
        board = []
        control = []
        diagnostics = []
        node.create_subscription(McuState, 'mcu/state', lambda message: board.append(message), 1)
        node.create_subscription(ControlStatus, 'control/status', lambda message: control.append(message), 1)

        def diagnostic(message):
            diagnostics.extend(status for status in message.status if status.name == 'mcu_serial')

        node.create_subscription(DiagnosticArray, 'diagnostics', diagnostic, 10)
        velocity = node.create_publisher(TwistStamped, 'cmd_vel', 1)
        enable = node.create_client(SetBool, 'control/set_enabled')

        def wait(predicate, timeout=6, stream=False):
            nonlocal next_navigation
            deadline = time.monotonic() + timeout
            next_command = 0.
            while time.monotonic() < deadline:
                now = time.monotonic()
                if peers:
                    master, _ = peers[0]
                    if now >= next_navigation:
                        try:
                            os.write(master, b'$GNGGA,navigation-protocol-is-not-MCU\r\n')
                        except OSError as error:
                            if error.errno not in (errno.EAGAIN, errno.EIO):
                                raise
                        next_navigation = now + .05
                    try:
                        received_on_navigation.extend(os.read(master, 4096))
                    except OSError as error:
                        if error.errno not in (errno.EAGAIN, errno.EIO):
                            raise
                if stream and now >= next_command:
                    message = TwistStamped()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = 'base_link'
                    message.twist.linear.x = .1
                    velocity.publish(message)
                    next_command = now + .02
                rclpy.spin_once(node, timeout_sec=.003)
                if predicate():
                    return
            raise AssertionError(f'timeout: diagnostics={diagnostics[-3:]}; control={control[-1:]}')

        def pump(duration, **kwargs):
            end = time.monotonic() + duration
            wait(lambda: time.monotonic() >= end, timeout=duration + 1, **kwargs)

        try:
            launch('terramind_control', 'control_manager_node')
            serial = driver()
            wait(lambda: any('未发现候选串口' in item.message for item in diagnostics))
            terminate(serial)

            master, slave = os.openpty()
            peers.append((master, slave))
            tty.setraw(slave)
            os.set_blocking(master, False)
            (ports / 'ttyUSB0-navigation').symlink_to(os.ttyname(slave))
            first_path = ports / 'ttyUSB1-control'
            second_path = ports / 'ttyACM0-another-board'
            first = emulator(first_path)
            second = emulator(second_path)
            alias = preferred / 'control-board'
            alias.symlink_to(first_path)
            marker = len(diagnostics)
            serial = driver()
            wait(lambda: any('发现多个控制板' in item.message for item in diagnostics[marker:]))
            pump(.4)
            assert not board, 'ambiguous boards must never enter the control session'
            assert not received_on_navigation, 'discovery wrote bytes to the navigation port'

            terminate(second)
            wait(lambda: board and board[-1].link_ready and control and control[-1].link_ready)
            wait(lambda: any(item.hardware_id == str(alias) for item in diagnostics[marker:]))
            assert board[-1].simulated and board[-1].mode == 2 and not control[-1].enabled
            assert board[-1].chassis.left_target_rpm == 0
            old_connection = board[-1].connection_id
            assert enable.wait_for_service(timeout_sec=3)
            request = SetBool.Request()
            request.data = True
            future = enable.call_async(request)
            wait(future.done, stream=True)
            assert future.result().success, future.result().message
            wait(lambda: board[-1].mode == 1 and abs(board[-1].chassis.linear_mps - .1) < 1e-5, stream=True)
            pump(.2, stream=True)

            terminate(first)
            wait(lambda: not control[-1].enabled and not control[-1].link_ready)
            alias.unlink()
            new_path = ports / 'ttyUSB9-renumbered'
            emulator(new_path)
            wait(lambda: board[-1].connection_id != old_connection and board[-1].link_ready
                 and control[-1].link_ready)
            wait(lambda: diagnostics[-1].hardware_id == str(new_path))
            pump(.2, stream=True)
            assert not control[-1].enabled and board[-1].mode == 2
            assert board[-1].chassis.left_target_rpm == 0 and board[-1].chassis.right_target_rpm == 0
            assert not received_on_navigation, 'driver wrote control bytes to an unrelated device'
            print('PASS automatic serial: no ports, alias deduplication, ambiguous boards, passive navigation filtering, handshake, control, unplug/renumber, re-enable required')
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            node.destroy_node()
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
            for master, slave in peers:
                os.close(master)
                os.close(slave)
            log.close()


if __name__ == '__main__':
    main()
