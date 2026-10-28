"""米制画区画布与区域作业页面；鼠标只编辑几何，不直接发送运动命令。"""
import math
from PyQt5 import QtCore, QtGui, QtWidgets
from .coverage import rectangle_bounds


class CoverageCanvas(QtWidgets.QWidget):
    cornerChanged = QtCore.pyqtSignal(float,float)

    def __init__(self, runtime):
        super().__init__()
        self.runtime = runtime
        self.corner = (2.,2.)
        self.cutting_width = runtime.default_cutting_width
        self.span = 10.
        self.dragging = False
        self.setMinimumSize(500,350)
        self.setMouseTracking(True)

    @property
    def scale(self):
        return min(self.width(),self.height())/self.span

    def point(self,x,y):
        return QtCore.QPointF(self.width()/2+x*self.scale,self.height()/2-y*self.scale)

    def mousePressEvent(self,event):
        if event.button() == QtCore.Qt.LeftButton and self.runtime.anchor and self.runtime.node.phase == 'idle':
            self.dragging = True
            self.mouseMoveEvent(event)

    def mouseMoveEvent(self,event):
        if self.dragging:
            x=(event.x()-self.width()/2)/self.scale
            y=(self.height()/2-event.y())/self.scale
            # 拖动实际区域的远角，将刀盘前置距离及侧缘半割幅还原成区域尺寸。
            length=max(.001,abs(x)-self.runtime.mower_forward_offset)
            breadth=2*abs(y) if abs(y)<=self.cutting_width/2 else abs(y)+self.cutting_width/2
            self.corner = (math.copysign(length,x),math.copysign(max(.001,breadth),y))
            self.cornerChanged.emit(*self.corner)
            self.update()

    def mouseReleaseEvent(self,event):
        self.dragging = False

    def wheelEvent(self,event):
        self.span = max(.5,min(1000.,self.span*(.8 if event.angleDelta().y()>0 else 1.25)))
        self.update()

    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(),QtGui.QColor('#f5f9fa'))
        grid=10**math.floor(math.log10(self.span/7))
        p.setPen(QtGui.QPen(QtGui.QColor('#dce7e9'),1))
        for i in range(-int(self.span/grid),int(self.span/grid)+1):
            x=i*grid
            p.drawLine(self.point(x,-self.span),self.point(x,self.span))
            p.drawLine(self.point(-self.span,x),self.point(self.span,x))
        p.setPen(QtGui.QColor('#526c78'))
        p.drawText(12,22,f'网格 {grid:g} m · 滚轮缩放 · X 沿建区时车头方向')
        runtime=self.runtime
        if runtime.anchor is None:
            p.drawText(self.rect(),QtCore.Qt.AlignCenter,'等待有效定位，点击“以当前位置建区”')
            return
        bounds=runtime.plan.bounds if runtime.plan else rectangle_bounds(
            *self.corner,self.cutting_width,runtime.mower_forward_offset)
        rect=QtCore.QRectF(self.point(*bounds[:2]),self.point(*bounds[2:])).normalized()
        p.setPen(QtGui.QPen(QtGui.QColor('#147f8a'),2))
        p.setBrush(QtGui.QColor(25,160,160,25));p.drawRect(rect)
        if runtime.plan:
            for segment,points in zip(runtime.plan.segments,runtime.plan.preview()):
                if segment.kind == 'rotate':continue
                color='#0d9987' if segment.mowing else ('#d58028' if segment.phase=='return' else '#6b8db0')
                p.setPen(QtGui.QPen(QtGui.QColor(color),2.5));p.setBrush(QtCore.Qt.NoBrush)
                path=QtGui.QPainterPath()
                for i,pose in enumerate(points):
                    pt=self.point(*runtime.plan.local(pose.x,pose.y))
                    path.moveTo(pt) if i==0 else path.lineTo(pt)
                p.drawPath(path)
        p.setPen(QtGui.QPen(QtGui.QColor('#334654'),2));p.setBrush(QtGui.QColor('#ffffff'))
        p.drawEllipse(self.point(0,0),5,5)
        p.drawText(self.point(0,0)+QtCore.QPointF(8,-8),'首行起点 / 返航点 →')
        try:
            pose,_=runtime.pose()
            a=runtime.anchor;c,s=math.cos(a.yaw),math.sin(a.yaw)
            dx,dy=pose.x-a.x,pose.y-a.y
            x,y=c*dx+s*dy,-s*dx+c*dy
            yaw=pose.yaw-a.yaw
            p.setPen(QtGui.QPen(QtGui.QColor('#c13d51'),3));p.setBrush(QtGui.QColor('#c13d51'))
            p.drawEllipse(self.point(x,y),4,4)
            p.drawLine(self.point(x,y),self.point(x+.25*math.cos(yaw),y+.25*math.sin(yaw)))
        except ValueError:
            pass


class CoverageWidget(QtWidgets.QWidget):
    def __init__(self,node,draft):
        super().__init__()
        self.runtime=node.coverage
        self.draft=draft
        layout=QtWidgets.QVBoxLayout(self)
        top=QtWidgets.QHBoxLayout()
        self.source=QtWidgets.QComboBox()
        self.source.addItem('导航板 · 实机定位','navigation')
        self.source.addItem('二维仿真 · 模型真值','simulation')
        self.source.setCurrentIndex(1 if self.runtime.source=='simulation' else 0)
        self.capture=QtWidgets.QPushButton('以当前位置建区')
        top.addWidget(self.source);top.addWidget(self.capture)
        top.addStretch();layout.addLayout(top)
        self.canvas=CoverageCanvas(self.runtime);layout.addWidget(self.canvas,1)
        inputs=QtWidgets.QHBoxLayout()
        def number(title,low,high,value,unit):
            inputs.addWidget(QtWidgets.QLabel(title))
            field=QtWidgets.QDoubleSpinBox();field.setRange(low,high);field.setDecimals(3)
            field.setValue(value);field.setSuffix(unit);field.setSingleStep(.05)
            field.setKeyboardTracking(False);inputs.addWidget(field)
            return field
        self.x=number('区域长度 X',-500,500,2.,' m')
        self.y=number('区域宽度 Y',-500,500,2.,' m')
        self.width=number('有效割幅',.05,10.,self.runtime.default_cutting_width,' m')
        self.speed=number('速度指令',max(.02,self.runtime.minimum_linear),self.runtime.speed_limit,
                          min(.15,self.runtime.speed_limit),' m/s')
        layout.addLayout(inputs)
        inputs=QtWidgets.QHBoxLayout()
        self.start_position=number('启动位置容差',.001,.5,self.runtime.start_position_tolerance,' m')
        self.start_yaw=number('启动朝向容差',.01,30.,math.degrees(self.runtime.start_yaw_tolerance),' °')
        self.start_position.setSingleStep(.01)
        self.start_yaw.setSingleStep(.5)
        layout.addLayout(inputs)
        tolerance_hint=QtWidgets.QLabel('启动容差仅用于与建区位姿比较，修改立即生效；不改变行驶到位精度。重开面板恢复默认值。')
        tolerance_hint.setWordWrap(True);layout.addWidget(tolerance_hint)
        hint=QtWidgets.QLabel('绿色：覆盖 · 蓝色：进入/掉头 · 橙色：返航 · 红色：当前位姿\n'
                             '正 X：首行沿车头直走；负 X：先转向车后。Y 正为左侧、负为右侧。区域按刀盘位置和半割幅偏移，尺寸不变。\n'
                             f'路线为后轴中心轨迹；刀盘中心在后轴前方 {self.runtime.mower_forward_offset:.2f} m，已补偿作业行端点。\n'
                             f'换行方式：{"停转移位" if self.runtime.turn_style == "pivot" else "半圆弧"}。\n'
                             '允许掉头越出覆盖区。刀盘和升降使用手动页的勾选/设定；换行与返航关闭刀盘，升降保持。')
        hint.setWordWrap(True);layout.addWidget(hint)
        self.tool_summary=QtWidgets.QLabel();layout.addWidget(self.tool_summary)
        bottom=QtWidgets.QHBoxLayout()
        self.generate=QtWidgets.QPushButton('生成路线')
        self.execute=QtWidgets.QPushButton('开始区域作业');self.execute.setObjectName('start')
        self.status=QtWidgets.QLabel();self.status.setWordWrap(True)
        bottom.addWidget(self.generate);bottom.addWidget(self.execute);bottom.addWidget(self.status,1)
        layout.addLayout(bottom)
        self.capture.clicked.connect(lambda:self.call(self.runtime.capture))
        self.generate.clicked.connect(lambda:self.call(self.generate_plan))
        self.execute.clicked.connect(lambda:self.call(lambda:self.runtime.start(self.draft(),self.speed.value())))
        self.source.currentIndexChanged.connect(self.change_source)
        self.canvas.cornerChanged.connect(self.drag)
        for field in (self.x,self.y,self.width):field.valueChanged.connect(self.edited)
        for field in (self.start_position,self.start_yaw):
            field.valueChanged.connect(lambda:self.call(lambda:self.runtime.set_start_tolerances(
                self.start_position.value(),math.radians(self.start_yaw.value()))))

    def call(self,operation):
        try:operation()
        except (ValueError,RuntimeError) as error:
            self.runtime.message=str(error)
            self.runtime.node.note('区域作业：'+str(error))
        self.refresh()

    def generate_plan(self):
        self.runtime.generate(self.x.value(),self.y.value(),self.width.value())
        self.canvas.span=max(1.,2.8*max(abs(self.x.value()),abs(self.y.value()),self.width.value()))

    def change_source(self):
        self.runtime.source=self.source.currentData()
        self.runtime.anchor=self.runtime.plan=self.runtime.tracker=None
        self.runtime.message='定位来源已改变，请重新建区'
        self.refresh()

    def drag(self,x,y):
        self.x.setValue(x);self.y.setValue(y)

    def edited(self):
        self.runtime.plan=None
        self.runtime.message='矩形或宽度已修改，请生成路线'
        self.canvas.corner=(self.x.value(),self.y.value())
        self.canvas.cutting_width=self.width.value()
        self.canvas.update()

    def refresh(self):
        r=self.runtime;idle=r.node.phase=='idle' and r.node.enable_future is None
        for widget in (self.source,self.capture,self.x,self.y,self.width,self.speed,self.start_position,self.start_yaw):widget.setEnabled(idle)
        self.generate.setEnabled(idle and r.anchor is not None)
        self.execute.setEnabled(idle and r.plan is not None)
        try:r.pose();error=''
        except ValueError as e:error=str(e)
        self.capture.setEnabled(idle and not error)
        self.execute.setEnabled(self.execute.isEnabled() and not error)
        self.status.setText(error or r.message)
        tools=self.draft(False)
        blade=f'刀盘 {tools.mower_percent:.1f} %' if tools.mower_on else '刀盘关闭（仅行驶）'
        lift=f'升降保持 {tools.lift_mm:.2f} mm' if tools.lift_on else '升降关闭（回零）'
        self.tool_summary.setText(f'启动设定：{blade} · {lift} · 到位容差 {r.position_tolerance:.3f} m / {math.degrees(r.yaw_tolerance):.1f}°')
        self.canvas.update()
