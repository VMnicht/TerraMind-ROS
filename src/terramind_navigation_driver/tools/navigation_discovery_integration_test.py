#!/usr/bin/env python3
"""私有 PTY 和隔离 ROS 域：双驱动发现、无效定位、歧义及拔插重连。不访问物理串口。"""
import errno
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import tty
import yaml
from navigation_integration_test import encode


class Peer:
    def __init__(self, path, status=0x94, corrupt=False):
        self.path = path
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        path.symlink_to(os.ttyname(self.slave))
        self.status, self.corrupt, self.sequence = status, corrupt, 0
        self.streaming = True
        self.next_send = 0.
        self.received = bytearray()

    def tick(self):
        if self.master is None:
            return
        try:
            self.received.extend(os.read(self.master, 4096))
        except OSError as error:
            if error.errno not in (errno.EAGAIN, errno.EIO):
                raise
        if self.streaming and time.monotonic() >= self.next_send:
            self.sequence += 1
            data = encode(self.sequence, 1000+self.sequence*.01, status=self.status)
            if self.corrupt:
                data[30] ^= 1
            try:
                os.write(self.master, data)
            except OSError as error:
                if error.errno not in (errno.EAGAIN, errno.EIO):
                    raise
            self.next_send = time.monotonic()+.01

    def close(self):
        if self.master is not None:
            os.close(self.master); os.close(self.slave)
            self.master = self.slave = None
            self.path.unlink(missing_ok=True)


def main():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    from terramind_interfaces.msg import NavigationStatus, McuState
    import rclpy

    processes, peers = [], []
    with tempfile.TemporaryDirectory(prefix='terramind-nav-auto-') as directory:
        root = Path(directory)
        ports = root/'ports'; ports.mkdir()
        os.environ['ROS_LOG_DIR'] = str(root/'ros-log')
        log_path = root/'children.log'
        log = log_path.open('w+')
        params = {'device': 'auto', 'discovery_patterns': [str(ports/'*')],
                  'discovery_listen_s': .3, 'reconnect_interval_s': .15}
        config = root/'config.yaml'
        config.write_text(yaml.safe_dump({
            'navigation_serial_node': {'ros__parameters': {
                **params, 'allow_simulated': True, 'connection_timeout_s': .4}},
            'mcu_serial_node': {'ros__parameters': {**params, 'simulated': True,
                'discovery_patterns': list(params['discovery_patterns'])}},
        }))
        rclpy.init()
        node = rclpy.create_node('navigation_discovery_integration_test')
        statuses, boards, diagnostics = [], [], []
        node.create_subscription(NavigationStatus, 'navigation/status', statuses.append, 10)
        node.create_subscription(McuState, 'mcu/state', boards.append, 10)
        node.create_subscription(DiagnosticArray, 'diagnostics',
                                 lambda msg: diagnostics.extend(msg.status), 10)

        def launch(package, name, *args):
            binary = Path(get_package_prefix(package))/'lib'/package/name
            process = subprocess.Popen([str(binary), *args], stdout=log, stderr=log)
            processes.append(process)
            return process

        def nav(device='auto'):
            return launch('terramind_navigation_driver', 'navigation_serial_node', '--ros-args',
                          '--params-file', str(config), '-p', f'device:={device}')

        def wait(predicate, timeout=8):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                for peer in peers:
                    peer.tick()
                    assert not peer.received, f'bytes sent to navigation candidate: {peer.path}'
                rclpy.spin_once(node, timeout_sec=.002)
                if predicate():
                    return
            raise AssertionError(f'timeout; navigation={statuses[-1:]}; diagnostics={diagnostics[-4:]}')

        def pump(seconds):
            end = time.monotonic()+seconds
            wait(lambda: time.monotonic() >= end, seconds+1)

        def stop(process):
            process.send_signal(signal.SIGINT)
            process.wait(timeout=3)
            assert process.returncode == 0

        try:
            navigation = nav()
            wait(lambda: statuses and '未发现候选串口' in statuses[-1].reason)
            first = Peer(ports/'nav-a'); peers.append(first)
            second = Peer(ports/'nav-b'); peers.append(second)
            bad = Peer(ports/'noise', corrupt=True); peers.append(bad)
            alias = ports/'nav-a-alias'; alias.symlink_to(first.path)
            launch('terramind_sim', 'serial_board_emulator', '--link', str(ports/'control'))
            launch('terramind_mcu', 'mcu_serial_node', '--ros-args', '--params-file', str(config))
            wait(lambda: any('发现多个导航板' in s.reason for s in statuses))
            assert not any(s.online for s in statuses), 'ambiguous navigation must not connect'
            wait(lambda: boards and boards[-1].link_ready)
            assert boards[-1].mode == 2 and boards[-1].chassis.linear_mps == 0
            second.close()
            wait(lambda: statuses[-1].online)
            assert not statuses[-1].usable, 'initialization frames must connect without usable navigation'
            first.status = 0x9D
            wait(lambda: statuses[-1].usable and statuses[-1].origin_ready)
            origin = (statuses[-1].origin_latitude_rad, statuses[-1].origin_longitude_rad,
                      statuses[-1].origin_height_m)
            wait(lambda: any(d.name == 'navigation_serial' and d.hardware_id == str(first.path)
                             for d in diagnostics))
            connection = boards[-1].connection_id

            # 静默触发自动模式重新扫描；新鲜的定位无效帧不会触发重新连接。
            first.status = 0x94
            wait(lambda: not statuses[-1].usable and statuses[-1].online)
            session = statuses[-1].session_id
            pump(.8)
            assert statuses[-1].online and statuses[-1].session_id == session
            first.streaming = False
            wait(lambda: not statuses[-1].online)
            first.streaming = True; first.status = 0x9D
            wait(lambda: statuses[-1].usable and statuses[-1].session_id != session)

            # 拔插后路径变化；保留 PC 外参及原点，控制板连接不受影响。
            session = statuses[-1].session_id
            first.close(); alias.unlink()
            wait(lambda: not statuses[-1].online)
            replacement = Peer(ports/'nav-renumbered', status=0x9D); peers.append(replacement)
            wait(lambda: statuses[-1].usable and statuses[-1].session_id != session)
            wait(lambda: any(d.name == 'navigation_serial' and d.hardware_id == str(replacement.path)
                             for d in diagnostics))
            assert (statuses[-1].origin_latitude_rad, statuses[-1].origin_longitude_rad,
                    statuses[-1].origin_height_m) == origin
            assert boards[-1].connection_id == connection and boards[-1].link_ready

            # 显式路径跳过发现，即使存在第二块同协议设备也只连接指定设备。
            stop(navigation)
            extra = Peer(ports/'nav-extra', status=0x9D); peers.append(extra)
            statuses.clear(); diagnostics.clear()
            navigation = nav(str(replacement.path))
            wait(lambda: statuses and statuses[-1].usable)
            wait(lambda: any(d.name == 'navigation_serial' and d.hardware_id == str(replacement.path)
                             for d in diagnostics))
            stop(navigation)
            statuses.clear()
            navigation = nav(str(ports/'missing-manual'))
            pump(.7)
            assert statuses and not any(s.online for s in statuses), 'manual mode must not fall back to scan'
            stop(navigation)

            # 长扫描仍持续发布状态，SIGINT 不必等扫描窗口结束。
            statuses.clear()
            navigation = launch('terramind_navigation_driver', 'navigation_serial_node', '--ros-args',
                                '--params-file', str(config), '-p', 'discovery_listen_s:=10.0')
            wait(lambda: len(statuses) >= 5 and '正在扫描' in statuses[-1].reason)
            started = time.monotonic(); stop(navigation)
            assert time.monotonic()-started < 1.5
            print('PASS navigation auto: absent/ambiguous/alias, concurrent control discovery, '
                  'invalid fix, receive-only, silence recovery, unplug/renumber, preserved origin, '
                  'manual override, scan status and cancellation')
        except Exception:
            log.flush()
            print(log_path.read_text())
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            for peer in peers:
                peer.close()
            node.destroy_node(); rclpy.shutdown()
            log.close()


if __name__ == '__main__':
    main()
