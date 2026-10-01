#!/usr/bin/env python3
"""Isolated ROS process integration test. Only launches simulated backends."""
import argparse
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['sim', 'serial'], required=True)
    a = p.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(40, 100) if a.mode == 'sim' else random.randint(110, 200))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped
    from terramind_interfaces.msg import McuState, ControlStatus, ImplementCommand
    from nav_msgs.msg import Odometry
    from std_srvs.srv import SetBool, Trigger

    processes = []
    with tempfile.TemporaryDirectory(prefix='terramind-ros-test-') as directory:
        os.environ['ROS_LOG_DIR'] = str(Path(directory)/'ros_logs')
        log = open(Path(directory)/'children.log', 'w+')

        def launch(package, executable, *args):
            path = Path(get_package_prefix(package))/'lib'/package/executable
            process = subprocess.Popen([str(path), *args], stdout=log, stderr=log)
            processes.append(process)
            return process

        rclpy.init()
        node = Node('terramind_integration_test')
        board = []; control = []; poses = []
        node.create_subscription(McuState, 'mcu/state', lambda m: board.append(m), 1)
        node.create_subscription(ControlStatus, 'control/status', lambda m: control.append(m), 1)
        node.create_subscription(Odometry, 'sim/ground_truth', lambda m: poses.append(m), 10)
        pub = node.create_publisher(TwistStamped, 'cmd_vel', 1)
        implement_pub = node.create_publisher(ImplementCommand, 'implements/command', 1)
        enable = node.create_client(SetBool, 'control/set_enabled')
        stop = node.create_client(Trigger, 'control/stop')

        def wait(predicate, timeout=5, stream=False, velocity=.12, implement=False):
            deadline = time.monotonic()+timeout
            next_send = 0
            while time.monotonic() < deadline:
                if stream and time.monotonic() >= next_send:
                    m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg();m.header.frame_id='base_link'
                    m.twist.linear.x = velocity;m.twist.angular.z=.2;pub.publish(m)
                    if implement:
                        im=ImplementCommand();im.header=m.header;im.left_on=True;im.left_rpm=120.;im.mower_on=True;im.mower_percent=5.;implement_pub.publish(im)
                    next_send = time.monotonic()+.02
                rclpy.spin_once(node, timeout_sec=.005)
                if predicate():
                    return
            raise AssertionError(f'timeout in {a.mode}: {control[-1] if control else "no control state"}')

        def pump(duration, **kwargs):
            end=time.monotonic()+duration
            wait(lambda: time.monotonic()>=end, timeout=duration+1, **kwargs)

        def call(client, request):
            assert client.wait_for_service(timeout_sec=5)
            future=client.call_async(request)
            wait(lambda: future.done(), timeout=3)
            assert future.exception() is None and future.result().success, str(future.result())

        def arm():
            assert board[-1].simulated, 'refusing to enable an unmarked backend'
            req=SetBool.Request();req.data=True;call(enable, req)

        try:
            emulator = None
            if a.mode == 'serial':
                link=str(Path(directory)/'mcu')
                emulator=launch('terramind_sim','serial_board_emulator','--link',link,'--fragment','3')
                backend=launch('terramind_mcu','mcu_serial_node','--ros-args','-p',f'device:={link}','-p','simulated:=true','-p','reconnect_interval_s:=0.2')
            else:
                backend=launch('terramind_sim','sim_interface_node')
            manager=launch('terramind_control','control_manager_node')
            wait(lambda: board and board[-1].link_ready and control and control[-1].link_ready and pub.get_subscription_count()>0)
            assert board[-1].mode==2 and not control[-1].enabled
            arm();wait(lambda: board[-1].mode==1 and abs(board[-1].chassis.linear_mps-.12)<1e-5 and board[-1].mower.on, stream=True, implement=True)
            pump(.3,stream=True,implement=True)
            if a.mode=='sim':
                assert poses and poses[-1].pose.pose.position.x>0
            # Stop publishing implements while keeping cmd_vel alive: independent watchdog.
            wait(lambda: not control[-1].enabled and board[-1].mode==2,stream=True)
            assert board[-1].chassis.left_target_rpm==0 and not board[-1].mower.on
            pump(.2,stream=True);assert board[-1].mode==2
            arm();wait(lambda: board[-1].mode==1 and board[-1].chassis.linear_mps>0,stream=True)
            wait(lambda: not control[-1].enabled and board[-1].mode==2)
            arm();wait(lambda: board[-1].mode==1,stream=True)
            call(stop,Trigger.Request());wait(lambda: board[-1].mode==2)
            pump(.2,stream=True);assert not control[-1].enabled
            arm();wait(lambda: board[-1].mode==1,stream=True)
            wait(lambda: not control[-1].enabled and board[-1].mode==2,stream=True,velocity=2.)
            arm();wait(lambda: board[-1].mode==1,stream=True)
            unsupported=ImplementCommand();unsupported.header.stamp=node.get_clock().now().to_msg()
            unsupported.lift_on=True;implement_pub.publish(unsupported)
            wait(lambda: not control[-1].enabled and board[-1].mode==2,stream=True)
            assert not board[-1].lift.on
            if a.mode=='sim':
                arm();wait(lambda: board[-1].mode==1,stream=True)
                drop=node.create_client(SetBool,'sim/drop_status');req=SetBool.Request();req.data=True;call(drop,req)
                pump(.4,stream=True);assert not control[-1].enabled
                req.data=False;call(drop,req)
                wait(lambda: board[-1].link_ready and control[-1].link_ready and board[-1].mode==2)
            else:
                old_id=board[-1].connection_id
                emulator.terminate();emulator.wait(timeout=3)
                wait(lambda: not control[-1].link_ready)
                emulator=launch('terramind_sim','serial_board_emulator','--link',link)
                wait(lambda: board[-1].connection_id!=old_id and board[-1].link_ready and control[-1].link_ready,timeout=6)
                assert not control[-1].enabled and board[-1].mode==2
            # Hard crash of the manager must be caught by the backend's own watchdog.
            arm();wait(lambda: board[-1].mode==1 and board[-1].chassis.linear_mps>0,stream=True)
            manager.kill();manager.wait(timeout=3)
            wait(lambda: board[-1].mode==2 and board[-1].chassis.left_target_rpm==0)
            assert backend.poll() is None
            print(f'PASS {a.mode}: handshake, drive/tools, independent input timeouts, stop latch, invalid command, link recovery, manager crash')
        except Exception:
            log.flush();log.seek(0);print(log.read())
            raise
        finally:
            node.destroy_node();rclpy.shutdown()
            for process in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in reversed(processes):
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait()
            log.close()


if __name__=='__main__':
    main()
