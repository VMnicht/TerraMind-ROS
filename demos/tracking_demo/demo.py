#!/usr/bin/env python3
"""独立故障注入仿真。无 ROS、串口、网络或真实车辆控制接口。"""
import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/terramind_panel'))
from terramind_panel.coverage import Pose2, plan_rectangle, wrap
from terramind_panel.coverage_tracking import CoverageTracker

DT = .02
PRESETS = {'明显蛇形偏离': 1.5, '转弯过冲 / 来回修正': 1.2}


@dataclass
class Simulation:
    label: str
    delay: float
    samples: list
    maximum_error: float
    completed: bool
    fault: str

    def summary(self):
        return dict(label=self.label, simulated=True, injected_delay_s=self.delay,
                    duration_s=self.samples[-1][0], maximum_cross_track_m=self.maximum_error,
                    completed=self.completed, fault=self.fault)


def simulate(plan, label, delay):
    pose=plan.origin
    tracker=CoverageTracker(plan,.1,.06,.07,.1,.6,.6)
    history=deque([pose]*(round(delay/DT)+1), maxlen=round(delay/DT)+1)
    left=right=maximum=0.
    samples=[]
    for tick in range(18001):
        now=tick*DT
        index=tracker.index
        if index<len(plan.segments):
            segment=plan.segments[index]
            if segment.kind=='line':
                dx,dy=pose.x-segment.start.x,pose.y-segment.start.y
                maximum=max(maximum,abs(-dx*math.sin(segment.start.yaw)+dy*math.cos(segment.start.yaw)))
        command=tracker.update(history[0],now)
        samples.append((now,pose.x,pose.y,pose.yaw,index,maximum))
        if tracker.done or tracker.fault:break
        # 同一运动模型：32 cm 固件轮距换算、38 cm 实际轮距、左右响应差、死区和惯性。
        l=command.linear-.16*command.angular
        r=command.linear+.16*command.angular
        l=l*1.72 if abs(l)>=.06 else 0.
        r=r*1.78 if abs(r)>=.06 else 0.
        left+=(l-left)*.12;right+=(r-right)*.12
        v=(left+right)/2;omega=(right-left)/.38
        pose=Pose2(pose.x+v*math.cos(pose.yaw)*DT,
                   pose.y+v*math.sin(pose.yaw)*DT,wrap(pose.yaw+omega*DT))
        history.append(pose)
    fault=tracker.fault or ('' if tracker.done else '演示时间上限')
    return Simulation(label,delay,samples,maximum,tracker.done,fault)


def build(preset):
    plan=plan_rectangle(Pose2(0.,0.,0.),2.,2.,.27,.2,'pivot')
    return plan,simulate(plan,'现行控制器 · 正常仿真',.2),simulate(plan,'故障注入 · 差跟踪仿真',PRESETS[preset])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preset',choices=PRESETS,default=next(iter(PRESETS)))
    parser.add_argument('--check',action='store_true',help='只验证仿真，不打开窗口')
    parser.add_argument('--snapshot',type=Path,help='离屏保存全程对比截图后退出')
    args=parser.parse_args()
    plan,normal,poor=build(args.preset)
    if args.check:
        assert normal.completed and not normal.fault
        assert poor.fault and not poor.completed
        assert poor.maximum_error > 3*normal.maximum_error
        assert normal.samples[0][1:4] == poor.samples[0][1:4]
        print(json.dumps({'source':'simulation_fault_injection','normal':normal.summary(),'poor':poor.summary()},ensure_ascii=False,indent=2))
        return

    from PyQt5 import QtCore,QtGui,QtWidgets

    class Plot(QtWidgets.QWidget):
        def __init__(self,owner,which,color):
            super().__init__();self.owner=owner;self.which=which;self.color=QtGui.QColor(color)
            self.setMinimumSize(450,490)

        def paintEvent(self,event):
            o=self.owner;sim=o.normal if self.which==0 else o.poor
            p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
            p.fillRect(self.rect(),QtGui.QColor('#f8fafc'))
            scale=min((self.width()-80)/3.2,(self.height()-120)/3.2)
            def point(x,y):return QtCore.QPointF(55+(x+.4)*scale,self.height()-60-(y+.5)*scale)
            p.setPen(QtGui.QColor('#13263c'));font=p.font();font.setPointSize(13);font.setBold(True);p.setFont(font)
            p.drawText(20,30,sim.label)
            font.setPointSize(10);font.setBold(False);p.setFont(font)
            p.setPen(QtGui.QPen(QtGui.QColor('#e0e6ed'),1))
            for i in range(-1,7):
                a=i*.5;p.drawLine(point(a,-.5),point(a,2.5));p.drawLine(point(-.5,a),point(2.7,a))
            xmin,ymin,xmax,ymax=o.plan.bounds
            p.setBrush(QtGui.QColor(61,126,126,12));p.setPen(QtGui.QPen(QtGui.QColor('#8393a5'),1.5))
            p.drawRect(QtCore.QRectF(point(xmin,ymax),point(xmax,ymin)))
            p.setBrush(QtCore.Qt.NoBrush);p.setPen(QtGui.QPen(QtGui.QColor('#a6afb9'),1,QtCore.Qt.DashLine))
            for segment,points in zip(o.plan.segments,o.preview):
                if segment.kind=='rotate':continue
                path=QtGui.QPainterPath()
                for i,q in enumerate(points):
                    path.moveTo(point(q.x,q.y)) if i==0 else path.lineTo(point(q.x,q.y))
                p.drawPath(path)
            end=min(o.frame,len(sim.samples)-1)
            p.setPen(QtGui.QPen(self.color,2.6));path=QtGui.QPainterPath()
            for i in range(0,end+1,2):
                q=sim.samples[i];path.moveTo(point(q[1],q[2])) if i==0 else path.lineTo(point(q[1],q[2]))
            q=sim.samples[end];path.lineTo(point(q[1],q[2]));p.drawPath(path)
            p.setBrush(self.color);p.drawEllipse(point(q[1],q[2]),5,5)
            p.drawLine(point(q[1],q[2]),point(q[1]+.16*math.cos(q[3]),q[2]+.16*math.sin(q[3])))
            p.setPen(QtGui.QColor('#45566c'))
            status=('已完成并返回起点' if sim.completed else '已停机：'+sim.fault) if end==len(sim.samples)-1 else '仿真播放中'
            p.drawText(20,self.height()-31,f'测量延迟 {sim.delay:.1f} s   |   已出现的最大横向误差 {q[5]*100:.1f} cm')
            p.drawText(20,self.height()-10,status)
            p.drawText(point(0,0)+QtCore.QPointF(5,18),'起点')

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__();self.plan,self.normal,self.poor=plan,normal,poor
            self.preview=plan.preview();self.frame=0
            self.setWindowTitle('TerraMind — 差跟踪演示（纯仿真，不连接设备）')
            self.resize(1160,780)
            layout=QtWidgets.QVBoxLayout(self)
            title=QtWidgets.QLabel('轨迹跟踪故障注入演示 · 2 × 2 m')
            title.setStyleSheet('font-size:24px;font-weight:600;color:#17334d;padding:8px;');layout.addWidget(title)
            label=QtWidgets.QLabel('两侧使用同一规划、控制器和运动模型，仅右侧增加定位延迟。全部为计算生成的仿真数据，不是实车记录。')
            label.setWordWrap(True);label.setStyleSheet('background:#fff1d6;color:#754b13;padding:10px;');layout.addWidget(label)
            charts=QtWidgets.QHBoxLayout();self.plots=[Plot(self,0,'#147c83'),Plot(self,1,'#d45c3d')]
            for plot in self.plots:charts.addWidget(plot)
            layout.addLayout(charts,1)
            self.summary=QtWidgets.QLabel();layout.addWidget(self.summary)
            self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.slider.valueChanged.connect(self.seek);layout.addWidget(self.slider)
            bar=QtWidgets.QHBoxLayout();self.toggle=QtWidgets.QPushButton('暂停');self.toggle.clicked.connect(self.pause)
            replay=QtWidgets.QPushButton('从头播放');replay.clicked.connect(self.replay)
            complete=QtWidgets.QPushButton('查看全程轨迹');complete.clicked.connect(self.full)
            self.preset=QtWidgets.QComboBox();self.preset.addItems(PRESETS);self.preset.setCurrentText(args.preset);self.preset.currentTextChanged.connect(self.reload)
            self.speed=QtWidgets.QComboBox();self.speed.addItems(['1×','4×','8×','16×']);self.speed.setCurrentIndex(2)
            export=QtWidgets.QPushButton('导出仿真截图 / 数据');export.clicked.connect(self.export)
            for widget in [self.toggle,replay,complete,self.preset,QtWidgets.QLabel('播放速度'),self.speed,export]:bar.addWidget(widget)
            layout.addLayout(bar)
            footer=QtWidgets.QLabel('虚线：规划路线　蓝绿：正常仿真　橙红：故障注入仿真　|　此窗口没有任何实车控制接口')
            layout.addWidget(footer)
            self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.step);self.timer.start(30)
            self.update_range()

        def update_range(self):
            self.slider.setRange(0,max(len(self.normal.samples),len(self.poor.samples))-1)
            self.summary.setText(f'全程仿真最大横向误差：正常 {self.normal.maximum_error*100:.1f} cm / 故障注入 {self.poor.maximum_error*100:.1f} cm（同一误差计算口径）')

        def seek(self,value):
            self.frame=value
            for plot in self.plots:plot.update()

        def step(self):
            count=max(1,round(.03/DT*int(self.speed.currentText()[:-1])))
            self.slider.setValue(min(self.slider.maximum(),self.frame+count))
            if self.frame==self.slider.maximum():self.timer.stop();self.toggle.setText('播放')

        def pause(self):
            if self.timer.isActive():self.timer.stop();self.toggle.setText('播放')
            else:
                if self.frame==self.slider.maximum():self.slider.setValue(0)
                self.timer.start(30);self.toggle.setText('暂停')

        def replay(self):self.slider.setValue(0);self.timer.start(30);self.toggle.setText('暂停')
        def full(self):self.timer.stop();self.slider.setValue(self.slider.maximum());self.toggle.setText('播放')
        def reload(self,preset):
            self.plan,self.normal,self.poor=build(preset);self.preview=self.plan.preview();self.update_range();self.replay()

        def export(self):
            from datetime import datetime
            directory=ROOT/'log/tracking-demo'/datetime.now().strftime('%Y%m%d-%H%M%S-%f');directory.mkdir(parents=True)
            self.full();app.processEvents();self.grab().save(str(directory/'simulation-comparison.png'))
            report={'source':'simulation_fault_injection','not_real_vehicle_measurement':True,
                    'normal':self.normal.summary(),'poor':self.poor.summary(),
                    'sample_columns':['time_s','x_m','y_m','yaw_rad','segment_index','running_max_cross_track_m'],
                    'normal_samples':self.normal.samples,'poor_samples':self.poor.samples}
            (directory/'simulation-data.json').write_text(json.dumps(report,ensure_ascii=False))
            QtWidgets.QMessageBox.information(self,'已导出仿真演示',str(directory))

    app=QtWidgets.QApplication([sys.argv[0]]);app.setStyle('Fusion')
    app.setFont(QtGui.QFont('Noto Sans CJK SC',10))
    window=Window();window.show()
    if args.snapshot:
        window.full();app.processEvents();args.snapshot.parent.mkdir(parents=True,exist_ok=True)
        assert window.grab().save(str(args.snapshot))
        print(args.snapshot)
        return
    return app.exec_()


if __name__=='__main__':
    sys.exit(main())
