#!/usr/bin/env python3
"""区域作业真实 Qt/ROS 联调，只启动私有二维仿真，不访问硬件。"""
import os
import argparse
from collections import deque
import signal
import subprocess
import tempfile
import time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose-source',choices=['simulation','navigation'],default='simulation')
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='87'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    os.environ['QT_QPA_PLATFORM']='offscreen'
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from PyQt5 import QtWidgets, QtCore, QtGui, QtTest
    from terramind_panel.controller import PanelNode
    from terramind_panel.window import PanelWindow
    from terramind_panel.coverage import wrap
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from terramind_interfaces.msg import NavigationStatus
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    app=QtWidgets.QApplication([])
    node=PanelNode();node.coverage.source=args.pose_source
    # 导航模式以延迟 200 ms 的模型测量发布真实接入使用的两种消息。
    # 字节解包、地理坐标和串口恢复另由 navigation_integration_test 验证。
    pending=deque()
    nav_usable=[True]
    if args.pose_source=='navigation':
        poses=node.create_publisher(PoseStamped,'navigation/pose',qos_profile_sensor_data)
        statuses=node.create_publisher(NavigationStatus,'navigation/status',1)
        def sample(m):pending.append((time.monotonic(),m))
        node.create_subscription(Odometry,'sim/ground_truth',sample,1)
        def publish_navigation():
            while pending and time.monotonic()-pending[0][0]>=.2:
                received,m=pending.popleft()
                status=NavigationStatus();status.header.stamp=node.get_clock().now().to_msg()
                status.header.frame_id='test_field';status.session_id=42
                status.online=True;status.usable=nav_usable[0];status.origin_ready=True
                status.extrinsics_calibrated=True;status.pose_child_frame='base_link'
                status.age_ms=int(1000*(time.monotonic()-received));status.simulated=True
                statuses.publish(status)
                pose=PoseStamped();pose.header=status.header;pose.pose=m.pose.pose
                poses.publish(pose)
        node.create_timer(.02,publish_navigation)
    window=PanelWindow(node);window.show()
    children=[]
    with tempfile.TemporaryDirectory(prefix='terramind-coverage-') as directory:
        log=open(Path(directory)/'children.log','w+')
        def launch(package,exe,*args):
            binary=Path(get_package_prefix(package))/'lib'/package/exe
            children.append(subprocess.Popen([str(binary),*args],stdout=log,stderr=log))
        def wait(predicate,timeout=8):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                app.processEvents()
                if predicate():return
                time.sleep(.002)
            raise AssertionError(f'{node.reason}; coverage={node.coverage.message}')
        try:
            launch('terramind_sim','sim_interface_node','--ros-args','-p','capabilities:=63')
            launch('terramind_control','control_manager_node')
            wait(lambda:not node.health_error() and 'simulation' in node.coverage.samples
                 and node.enable_client.service_is_ready())
            page=window.coverage_page
            window.mode_tabs.setCurrentWidget(page)
            app.processEvents()
            window.refresh()
            wait(lambda:len(node.coverage.histories.get(args.pose_source,()))>1 and
                 node.coverage.histories[args.pose_source][-1][0]-node.coverage.histories[args.pose_source][0][0] >= .45)
            page.capture.click()
            assert node.coverage.anchor is not None
            target=page.canvas.point(.8,.4).toPoint()
            QtTest.QTest.mousePress(page.canvas,QtCore.Qt.LeftButton,pos=page.canvas.point(0,0).toPoint())
            app.sendEvent(page.canvas,QtGui.QMouseEvent(QtCore.QEvent.MouseMove,
                          QtCore.QPointF(target),QtCore.Qt.NoButton,QtCore.Qt.LeftButton,QtCore.Qt.NoModifier))
            QtTest.QTest.mouseRelease(page.canvas,QtCore.Qt.LeftButton,pos=target)
            assert abs(page.x.value()-.8)<.04 and abs(page.y.value()-.4)<.04,(page.x.value(),page.y.value())
            page.x.setValue(.8);page.y.setValue(.4);page.width.setValue(.2)
            window.checks['mower'].setChecked(True);window.editors['mower_percent'].setValue(8.)
            window.checks['lift'].setChecked(True);window.editors['lift_mm'].setValue(5.)
            page.generate.click()
            assert node.coverage.plan is not None
            window.grab().save('/tmp/terramind-coverage-plan.png')
            page.execute.click()
            wait(lambda:node.phase=='running')
            assert node.board.simulated
            observed=set();mowed=False
            last_phase=None
            clock_offset=time.time()-time.monotonic()
            end=time.monotonic()+100
            while time.monotonic()<end and not node.coverage.tracker.done:
                app.processEvents();time.sleep(.002)
                tracker=node.coverage.tracker
                offset=time.time()-time.monotonic()
                if abs(offset-clock_offset)>.05:
                    print(f'CLOCK STEP: {offset-clock_offset:+.3f}s',flush=True)
                clock_offset=offset
                if tracker.phase!=last_phase:
                    print(f'PHASE {tracker.phase} segment={tracker.index}',flush=True)
                    last_phase=tracker.phase
                observed.add(tracker.phase)
                assert not tracker.fault,(tracker.fault,node.reason,list(node.events)[-5:])
                mowed=mowed or node.board.mower.on
                if tracker.phase=='return':
                    assert node.active.lift_on and node.active.lift_mm==5.
                    assert not node.active.mower_on
            assert node.coverage.tracker.done,node.coverage.message
            wait(lambda:node.phase=='idle' and not node.control.enabled and node.board.mode==2)
            pose,_=node.coverage.pose()
            assert pose.distance(node.coverage.anchor)<=.03
            assert abs(wrap(pose.yaw-node.coverage.anchor.yaw))<=.035
            actual=node.coverage.samples['simulation'][0]
            assert actual.distance(node.coverage.anchor)<=.03
            assert abs(wrap(actual.yaw-node.coverage.anchor.yaw))<=.035
            assert mowed and {'coverage','turn','return','align'}<=observed
            print(f'PASS {args.pose_source}: Qt rectangle drag, ROS coverage, arcs, tool phases, return position/yaw and disarm',flush=True)
            if args.pose_source=='navigation':
                # 重新规划后主动失效，验证真实订阅回调触发停机且恢复不自动续跑。
                wait(lambda:len(node.coverage.histories['navigation'])>10)
                settle=time.monotonic()+.8;wait(lambda:time.monotonic()>settle)
                page.capture.click();page.generate.click();page.execute.click()
                wait(lambda:node.phase=='running')
                nav_usable[0]=False
                wait(lambda:node.phase=='idle' and bool(node.coverage.tracker.fault))
                nav_usable[0]=True
                settle=time.monotonic()+.4;wait(lambda:time.monotonic()>settle)
                assert node.phase=='idle' and not node.coverage.active
                print('PASS navigation invalidation stops and recovery does not resume',flush=True)
        finally:
            node.stop('integration teardown')
            window.close_ready=True;window.close();node.destroy_node();rclpy.shutdown()
            for child in children:
                if child.poll() is None:child.send_signal(signal.SIGINT)
            for child in children:
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.wait()
            log.close()


if __name__=='__main__':
    main()
