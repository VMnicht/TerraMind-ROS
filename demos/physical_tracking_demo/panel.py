#!/usr/bin/env python3
"""实物差跟踪演示。独立入口，正常程序/配置不修改；刀盘等装置始终关闭。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from dataclasses import asdict
from datetime import datetime

ROOT=Path(__file__).resolve().parents[2]
from tracker import DemoTracker
from start_delay import DelayedStart


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-once',action='store_true',help='正常建区/启动检查后执行一次2×2m路线，停机并退出')
    parser.add_argument('--amplitude',type=float,default=.12,help='注入横向参考偏置，0～0.12 m')
    parser.add_argument('--start-delay',type=int,default=10,help='点击开始后的延时秒数，0～120，默认10')
    args=parser.parse_args()
    if not math.isfinite(args.amplitude) or not 0<=args.amplitude<=.12:parser.error('amplitude must be 0..0.12')
    if not 0<=args.start_delay<=120:parser.error('start-delay must be 0..120')
    os.environ.setdefault('ROS_DOMAIN_ID','0');os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from PyQt5 import QtWidgets,QtCore,QtGui
    from terramind_panel.controller import PanelNode
    from terramind_panel.window import PanelWindow
    from terramind_panel.coverage_widget import CoverageCanvas
    from terramind_panel.controls import Selection

    class DemoNode(PanelNode):
        def __init__(self):
            self.coverage_start_allowed=False
            self.delayed_start=None
            super().__init__()
        def stop(self,reason='已停止并取消启动倒计时',disable=False):
            if self.delayed_start:self.delayed_start.cancel()
            super().stop(reason,disable)
        def start(self,draft):
            if not self.coverage_start_allowed:raise ValueError('演示入口只允许区域作业，手动发送禁用')
            return super().start(draft)
        def _publish(self,s):
            # 无论 UI 或上游给什么作业装置目标，物理输出只保留有界底盘速度。
            super()._publish(Selection(linear_mps=max(0.,min(.1,s.linear_mps)),
                                       angular_radps=max(-.6,min(.6,s.angular_radps))))
        def tick(self):
            if self.phase!='idle' and self.board and any(getattr(self.board,k).on for k in
                    ('mower','lift','sprayer','left_spreader','right_spreader')):
                self.stop('演示检测到作业装置开启',disable=True)
            super().tick()
            if self.delayed_start:self.delayed_start.tick(time.monotonic())

    class TraceCanvas(CoverageCanvas):
        def paintEvent(self,event):
            super().paintEvent(event)
            if not node.coverage.plan:return
            painter=QtGui.QPainter(self);painter.setRenderHint(QtGui.QPainter.Antialiasing)
            painter.setPen(QtGui.QPen(QtGui.QColor('#cf3f42'),2.5))
            path=QtGui.QPainterPath();first=True
            for row in records[::max(1,len(records)//1200)]:
                if row['pose'] is None:continue
                pt=self.point(*node.coverage.plan.local(*row['pose'][:2]))
                if first:path.moveTo(pt);first=False
                else:path.lineTo(pt)
            painter.drawPath(path)

    class DemoWindow(PanelWindow):
        def closeEvent(self,event):
            if node.delayed_start:node.delayed_start.cancel()
            super().closeEvent(event)
        def refresh(self):
            super().refresh()
            p=self.coverage_page
            for widget in (p.x,p.y,p.width,p.speed,p.source,self.start_button,self.apply_button):
                widget.setEnabled(False)
            for check in self.checks.values():check.setEnabled(False)
            p.canvas.setEnabled(not node.delayed_start.pending)
            if node.delayed_start and node.delayed_start.pending:
                for widget in (p.capture,p.generate,p.execute):widget.setEnabled(False)
                p.status.setText(f'启动倒计时：{math.ceil(node.delayed_start.remaining(time.monotonic()))} 秒 · 保持停机 · 可点击停止全部取消')
            if hasattr(self,'delay_seconds'):
                self.delay_seconds.setEnabled(node.phase=='idle' and not node.delayed_start.pending)
                self.cancel_delay.setEnabled(node.delayed_start.pending)

    rclpy.init(args=['--ros-args','--params-file',str(ROOT/'src/terramind_bringup/config/control.yaml')],
                 signal_handler_options=SignalHandlerOptions.NO)
    app=QtWidgets.QApplication([sys.argv[0]]);app.setStyle('Fusion')
    app.setFont(QtGui.QFont('Noto Sans CJK SC',10))
    node=DemoNode();runtime=node.coverage
    original_start=runtime.start
    records=[];run=None;last_record=last_report=0.;cancelled=False;saved=False
    def start_demo(tools,speed):
        nonlocal run,records,last_record,last_report,saved
        if node.board is None or node.board.simulated or node.navigation is None or node.navigation.simulated:
            raise ValueError('该入口仅用于真实控制板和真实导航')
        if runtime.plan is None:raise ValueError('请先建区并生成路线')
        if (runtime.plan.corner_x,runtime.plan.corner_y,runtime.plan.width)!=(2.,2.,.27):
            raise ValueError('实物演示固定为2×2 m、割幅0.27 m')
        if runtime.turn_style!='pivot':raise ValueError('实物演示要求停转换行')
        tracker=DemoTracker(runtime.plan,.1,runtime.position_tolerance,runtime.yaw_tolerance,
                            runtime.minimum_linear,runtime.minimum_angular,runtime.maximum_angular,
                            amplitude=args.amplitude)
        directory=ROOT/'log/physical-tracking-demo'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        directory.mkdir(parents=True,exist_ok=False)
        # 所有正常的静止、身份、初始位姿、健康、唯一控制源和使能检查均由原入口执行。
        node.coverage_start_allowed=True
        try:original_start(Selection(),.1)
        finally:node.coverage_start_allowed=False
        runtime.tracker=tracker
        run=directory;records=[];last_record=last_report=0.;saved=False
        (run/'plan.json').write_text(json.dumps([asdict(s) for s in runtime.plan.segments]))
        (run/'metadata.json').write_text(json.dumps(dict(source='real_vehicle_fault_injection',
            demonstration_only=True,amplitude_m=args.amplitude,implements_forced_off=True,
            linear_command_limit_mps=.1,angular_command_limit_radps=.6,cross_track_stop_m=.18),indent=2))
        node.note('实物故障注入演示已开始；不代表正常控制效果')
    captured_plan=None
    def validate_delay():
        if node.phase!='idle' or node.enable_future is not None or (node.control and node.control.enabled):
            raise ValueError('车辆未处于未使能停机状态')
        if runtime.plan is None or runtime.plan is not captured_plan:
            raise ValueError('路线已改变，请重新开始')
        _,identity=runtime.pose()
        if identity!=runtime.identity:raise ValueError('定位原点或连接改变，请重新建区')
        conflict=node.input_conflict()
        if conflict:raise ValueError(conflict)
    def delay_failed(reason):
        runtime.message=reason;node.stop(reason,disable=True)
    delayed=DelayedStart(lambda:start_demo(Selection(),.1),validate_delay,delay_failed)
    node.delayed_start=delayed
    def request_start(tools,speed):
        nonlocal captured_plan
        if delayed.pending:raise ValueError('启动倒计时已经开始')
        captured_plan=runtime.plan
        delayed.arm(window.delay_seconds.value(),time.monotonic())
        node.note(f'启动倒计时 {window.delay_seconds.value()} 秒，期间保持停机')
    runtime.start=request_start
    window=DemoWindow(node);window.setWindowTitle('TerraMind · 实物差跟踪演示（故障注入）')
    banner=QtWidgets.QLabel('实物故障注入演示 · 非正常跟踪效果 · 2×2 m · 刀盘强制关闭 · 红线为真实定位轨迹')
    banner.setWordWrap(True);banner.setStyleSheet('background:#ffe2dc;color:#9f231d;font-weight:bold;padding:12px;')
    window.centralWidget().layout().insertWidget(0,banner)
    delay_row=QtWidgets.QHBoxLayout()
    delay_row.addWidget(QtWidgets.QLabel('启动延时'))
    window.delay_seconds=QtWidgets.QSpinBox();window.delay_seconds.setRange(0,120)
    window.delay_seconds.setValue(args.start_delay);window.delay_seconds.setSuffix(' 秒')
    delay_row.addWidget(window.delay_seconds)
    window.cancel_delay=QtWidgets.QPushButton('取消倒计时')
    window.cancel_delay.clicked.connect(lambda:delay_failed('已取消启动倒计时'))
    delay_row.addWidget(window.cancel_delay);delay_row.addStretch()
    window.centralWidget().layout().insertLayout(1,delay_row)
    page=window.coverage_page;old=page.canvas;page.canvas=TraceCanvas(runtime)
    page.layout().replaceWidget(old,page.canvas);old.deleteLater()
    page.canvas.cornerChanged.connect(page.drag)
    window.mode_tabs.setCurrentWidget(page);window.show()

    def record():
        nonlocal last_record,last_report,saved
        now=time.monotonic()
        if not run or now-last_record<.05:return
        last_record=now;b=node.board;t=runtime.tracker;sample=runtime.samples.get('navigation')
        pose=sample[0] if sample else None
        row=dict(t=now,phase=node.phase,index=t.index if t else None,message=runtime.message,
                 pose=[pose.x,pose.y,pose.yaw] if pose else None,v=node.active.linear_mps,
                 omega=node.active.angular_radps,left=b.chassis.left_actual_rpm if b else None,
                 right=b.chassis.right_actual_rpm if b else None,rtk=node.navigation.rtk_state if node.navigation else None,
                 max_cross_track=t.maximum_cross_track if t else None)
        records.append(row)
        if (not saved and t and (t.done or t.fault) and node.phase=='idle' and b and node.control
                and not node.control.enabled and abs(b.chassis.left_actual_rpm)<.5
                and abs(b.chassis.right_actual_rpm)<.5 and not b.mower.on):
            (run/'telemetry.json').write_text(json.dumps(records,ensure_ascii=False))
            (run/'result.json').write_text(json.dumps(dict(outcome='COMPLETE' if t.done else 'FAIL: '+t.fault,
                stopped=True,maximum_cross_track_m=t.maximum_cross_track,events=list(node.events)),ensure_ascii=False,indent=2))
            window.grab().save(str(run/'physical-demo.png'))
            saved=True
        if now-last_report>10:last_report=now;print(json.dumps(row,ensure_ascii=False),flush=True)
    timer=QtCore.QTimer();timer.timeout.connect(record);timer.start(50)
    def interrupt(*_):
        nonlocal cancelled
        cancelled=True;node.stop('用户停止实物演示',disable=True)
        if not args.run_once:window.close()
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    def pump():
        app.processEvents();time.sleep(.002)
        if cancelled:raise RuntimeError('用户已中止')
        if not window.isVisible():raise RuntimeError('窗口已关闭')
    outcome='窗口关闭'
    try:
        if args.run_once:
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                pump()
                try:
                    runtime.capture()
                    if node.input_conflict() or not node.enable_client.service_is_ready():continue
                    break
                except ValueError:pass
            else:raise RuntimeError('未满足正常建区条件：'+node.health_error())
            runtime.generate(2.,2.,.27);page.canvas.span=6.
            runtime.start(Selection(),.1)
            deadline=time.monotonic()+450+window.delay_seconds.value()
            while time.monotonic()<deadline:
                pump()
                if delayed.pending:continue
                if runtime.tracker is None:raise RuntimeError(node.reason)
                if runtime.tracker.fault:raise RuntimeError(runtime.tracker.fault)
                if runtime.tracker.done:outcome='COMPLETE';break
                if node.phase=='idle':raise RuntimeError(node.reason)
            else:raise RuntimeError('实物演示总时长上限')
        else:app.exec_()
    except Exception as error:outcome='FAIL: '+str(error)
    finally:
        node.stop('实物演示停止：'+outcome,disable=True)
        # 退出前独立泵 ROS 端点，确认停止应答，不依赖窗口是否已经关闭。
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            for _ in range(8):rclpy.spin_once(node,timeout_sec=0)
            node.tick();time.sleep(.005)
        b=node.board
        stopped=bool(b and node.control and not node.control.enabled and abs(b.chassis.left_actual_rpm)<.5
                     and abs(b.chassis.right_actual_rpm)<.5 and not b.mower.on)
        if run:
            (run/'telemetry.json').write_text(json.dumps(records,ensure_ascii=False))
            (run/'result.json').write_text(json.dumps(dict(outcome=outcome,stopped=stopped,
                maximum_cross_track_m=runtime.tracker.maximum_cross_track,events=list(node.events)),ensure_ascii=False,indent=2))
            window.grab().save(str(run/'physical-demo.png'))
        print('RESULT',outcome,'STOPPED',stopped,'LOG',run,flush=True)
        timer.stop();window.close_ready=True;window.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
