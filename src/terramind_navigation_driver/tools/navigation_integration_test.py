#!/usr/bin/env python3
"""私有 PTY + 独立 Python 编码 + 实际 ROS 导航节点；不访问物理串口。"""
import binascii
import math
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import tempfile
import time

FRAME = struct.Struct('<2sBBIdddf3f3f4fHH')


def encode(sequence, stamp, *, status=0x9D, age=200, latitude=.5, quaternion=None):
    q = quaternion if quaternion is not None else [math.sqrt(.5), 0, 0, math.sqrt(.5)]
    frame = bytearray(FRAME.pack(b'\xaa\x55', 1, status, sequence, stamp, latitude, 2., 20.,
                                 0., 1., 0., 0., 0., math.pi / 2, *q, age, 0))
    struct.pack_into('<H', frame, 78, binascii.crc_hqx(frame[2:78], 0xFFFF))
    return frame


def main():
    assert FRAME.size == 80
    assert binascii.crc_hqx(b'123456789', 0xFFFF) == 0x29B1
    assert encode(1, 1000)[78:] == b'\x3d\x23'
    # 每次测试隔离 ROS 域和 HOME 外日志，不继承正在运行的机器人节点。
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    workspace = tempfile.TemporaryDirectory(prefix='terramind-navigation-')
    os.environ['ROS_LOG_DIR'] = str(Path(workspace.name) / 'ros-log')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix
    from terramind_interfaces.msg import NavigationFrame, NavigationStatus
    from geometry_msgs.msg import PoseStamped, Vector3Stamped

    rclpy.init()
    node = rclpy.create_node('navigation_integration_test')
    status, raw, poses, velocities = [None], [], [], []
    subscriptions = [
        node.create_subscription(NavigationStatus, 'navigation/status', lambda m: status.__setitem__(0, m), 10),
        node.create_subscription(NavigationFrame, 'navigation/raw', raw.append, qos_profile_sensor_data),
        node.create_subscription(PoseStamped, 'navigation/pose', poses.append, qos_profile_sensor_data),
        node.create_subscription(Vector3Stamped, 'navigation/velocity', velocities.append, qos_profile_sensor_data),
    ]
    sequence, stamp = [1], [1000.]
    master = slave = None
    process = None
    with workspace as directory:
        link = Path(directory) / 'navigation'
        os.environ['ROS_LOG_DIR'] = str(Path(directory) / 'ros-log')
        log_path = Path(directory) / 'node.log'
        log = log_path.open('w+')
        try:
            master, slave = os.openpty()
            link.symlink_to(os.ttyname(slave))
            executable = Path(get_package_prefix('terramind_navigation_driver')) / 'lib/terramind_navigation_driver/navigation_serial_node'
            process = subprocess.Popen([str(executable), '--ros-args', '-p', f'device:={link}',
                                        '-p', 'allow_simulated:=true', '-p', 'reconnect_interval_s:=0.1'],
                                       stdout=log, stderr=subprocess.STDOUT)

            def drain(duration=.01):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    if process.poll() is not None:
                        log.flush()
                        raise AssertionError('driver exited: ' + log_path.read_text())
                    rclpy.spin_once(node, timeout_sec=min(.001, max(0., end-time.monotonic())))
                # 从端口不能收到驱动发出的任何数据（串口只接收，且禁用终端回显）。
                if select.select([master], [], [], 0)[0]:
                    unexpected = os.read(master, 4096)
                    assert not unexpected, f'driver unexpectedly transmitted {unexpected.hex()}'

            def send(**kwargs):
                sequence[0] = (sequence[0] + 1) & 0xFFFFFFFF
                stamp[0] += .01
                packet = encode(sequence[0], stamp[0], **kwargs)
                os.write(master, packet)
                return packet

            def wait(predicate, timeout=3., writer=None):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    if writer:
                        writer()
                    drain(.01)
                    if predicate():
                        return
                raise AssertionError(f'predicate timed out; status={status[0]}')

            # 首先等节点完成打开，避免 PTY 默认回显被误认为驱动写入。
            wait(lambda: status[0] is not None, writer=None)
            wait(lambda: status[0].usable and len(poses) > 0 and len(velocities) > 0, writer=send)
            assert status[0].origin_ready and status[0].simulated and status[0].rtk_state == 4
            assert status[0].pose_child_frame == 'navigation_imu'
            assert status[0].stamp_reference == 'receive_time'
            assert abs(poses[-1].pose.position.x) < 1e-8
            assert abs(poses[-1].pose.orientation.w - 1) < 1e-6
            assert abs(velocities[-1].vector.x - 1) < 1e-8
            assert raw[-1].header.stamp.sec > 1_000_000_000  # 不是协议的 1000 s
            origin = (status[0].origin_latitude_rad, status[0].origin_longitude_rad)

            # 任意分片、噪声、坏 CRC、未知版本、粘包恢复。
            corrupt = encode(100, stamp[0]); corrupt[30] ^= 1
            unknown = encode(101, stamp[0]); unknown[2] = 2
            os.write(master, b'noise' + corrupt + unknown)
            sequence[0] += 1; stamp[0] += .01
            packet = encode(sequence[0], stamp[0])
            for i in range(0, 80, 7):
                os.write(master, packet[i:i+7]); drain(.001)
            wait(lambda: status[0].crc_errors > 0 and status[0].version_errors > 0, writer=send)
            os.write(master, packet[:25]); drain(.15)
            wait(lambda: status[0].assembly_timeouts > 0)
            send(); send(); send()
            wait(lambda: status[0].usable, writer=send)

            # 重复时间戳不能持续刷新导航可用性。
            frozen_time = stamp[0]
            def duplicate():
                sequence[0] += 1
                os.write(master, encode(sequence[0], frozen_time))
            wait(lambda: not status[0].usable and status[0].duplicate_measurements > 0, writer=duplicate)
            assert status[0].online
            wait(lambda: status[0].usable, writer=send)

            # 无效状态帧保持在线但不发布位姿，年龄未知/非法数值也不能发布。
            wait(lambda: not status[0].usable, writer=lambda: send(status=0x94))
            drain(.05); count = len(poses)
            for _ in range(15):
                send(status=0x94); drain(.01)
            assert len(poses) == count and status[0].online
            for bad in ({'age': 65535}, {'age': 501}, {'quaternion': [0., 0., 0., 0.]},
                        {'latitude': float('nan')}, {'status': 0x9F}):
                for _ in range(6):
                    send(**bad); drain(.01)
                assert not status[0].usable and len(poses) == count
            wait(lambda: status[0].usable, writer=send)

            # 明确建立回绕前基准，随后 FFFFFFFF -> 0 不应触发重启。
            sequence[0] = 0xFFFFFFFD; stamp[0] += 1
            send(); drain(.08)
            session_id = raw[-1].session_id
            send(); drain(.02); send(); drain(.02); send(); drain(.02)
            assert raw[-1].session_id == session_id
            stamp[0] = 1; send(); drain(.08)
            assert raw[-1].session_id != session_id

            # 100/200 Hz 验证处理能力，不宣称为物理 UART 吞吐测量。
            for hz in (100, 200):
                count = len(raw)
                start = time.monotonic()
                for i in range(hz):
                    send()
                    drain(max(.0001, start + (i+1)/hz-time.monotonic()))
                drain(.03)
                assert len(raw) - count >= hz * .9, (hz, len(raw)-count)

            # 没有任何报文后应断流，再超时关闭并重连。
            reconnects = status[0].reconnects
            wait(lambda: not status[0].usable and status[0].online)
            wait(lambda: not status[0].online, timeout=4)
            wait(lambda: status[0].usable and status[0].reconnects > reconnects, writer=send)

            # 拔插到新的 PTY，稳定路径不变；会话改变但地理原点不重设。
            session_id = status[0].session_id
            os.close(master); os.close(slave)
            master = slave = None
            link.unlink()
            master, slave = os.openpty()
            link.symlink_to(os.ttyname(slave))
            # 新 PTY 初始回显阶段不写入数据。
            drain(.3)
            wait(lambda: status[0].usable and status[0].session_id != session_id,
                 writer=lambda: send(latitude=.500001))
            assert (status[0].origin_latitude_rad, status[0].origin_longitude_rad) == origin
            assert poses[-1].pose.position.y > 6
            print('PASS: golden CRC, receive-only PTY, ROS coordinates, fragments/noise/CRC/version, '
                  'timeout, invalid/status/duplicate data, wrap/reboot, 100/200 Hz, reconnect and fixed origin')
        finally:
            if process is not None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            for fd in (master, slave):
                if fd is not None:
                    os.close(fd)
            log.close()
            subscriptions.clear()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
