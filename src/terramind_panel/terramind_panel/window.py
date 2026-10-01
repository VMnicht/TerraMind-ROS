"""Operator panel. Edits are drafts until Start or Apply is clicked."""
import json
import time

import rclpy
from PyQt5 import QtCore, QtGui, QtWidgets

from .controls import ACTUATORS, Selection


STYLE = """
QMainWindow, QWidget#root { background: #eef3f5; color: #162f3b; }
QWidget { font-family: 'Noto Sans CJK SC'; font-size: 13px; }
QGroupBox { background: white; border: 1px solid #d7e1e6; border-radius: 9px;
 margin-top: 14px; padding: 15px 13px 10px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 5px; }
QLabel#heading { font-size: 25px; font-weight: 700; }
QLabel#muted { color: #546b78; }
QLabel#badge { padding: 7px 12px; border-radius: 7px; background: #dce8ed; font-weight: 600; }
QLabel#notice { padding: 9px; background: #e4edf2; border-radius: 6px; }
QPushButton { min-height: 31px; padding: 3px 13px; border: 1px solid #b9cbd4;
 border-radius: 6px; background: #f8fafb; color: #163441; }
QPushButton:hover { background: #e6f1f4; }
QPushButton#start { background: #087f8c; color: white; border: none; font-weight: 600; }
QPushButton#stop { background: #b62e3e; color: white; border: none; font-weight: 700; }
QPushButton:disabled { background: #e6ecef; color: #8799a1; border-color: #d6e0e4; }
QPushButton#start:disabled { background: #e6ecef; color: #8799a1; }
QDoubleSpinBox { min-height: 29px; border: 1px solid #bbcdd5; border-radius: 5px;
 background: #fbfdfe; padding: 3px 6px; }
QDoubleSpinBox:disabled { background: #eff2f4; color: #8c9ba2; }
QCheckBox::indicator { width: 18px; height: 18px; }
QTabWidget::pane { border: 1px solid #d7e1e6; background: white; border-radius: 5px; }
QTabBar::tab { padding: 8px 12px; background: #e6edef; }
QTabBar::tab:selected { background: white; color: #087f8c; }
QTableWidget { border: none; gridline-color: #e5edef; background: white; }
QHeaderView::section { border: none; padding: 7px; background: #edf3f6; color: #44606d; }
QPlainTextEdit { border: none; padding: 7px; background: white; }
"""


def label(text='', muted=False):
    widget = QtWidgets.QLabel(text)
    if muted:
        widget.setObjectName('muted')
    widget.setWordWrap(True)
    return widget


def spin(minimum, maximum, step, unit, decimals=2):
    widget = QtWidgets.QDoubleSpinBox()
    widget.setRange(minimum, maximum)
    widget.setSingleStep(step)
    widget.setDecimals(decimals)
    widget.setSuffix(' ' + unit)
    widget.setButtonSymbols(QtWidgets.QAbstractSpinBox.PlusMinus)
    widget.setKeyboardTracking(False)
    widget.setMinimumWidth(148)
    return widget


class PanelWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.closing = False
        self.close_ready = False
        self.editors = {}
        self.checks = {}
        self.support = {}
        self.last_events = None
        self.setWindowTitle('TerraMind · 控制与串口测试')
        self.resize(1200, 900)
        self.setStyleSheet(STYLE)
        root = QtWidgets.QWidget()
        root.setObjectName('root')
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(22, 16, 22, 16)
        outer.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        titles = QtWidgets.QVBoxLayout()
        title = label('TerraMind  /  控制测试')
        title.setObjectName('heading')
        titles.addWidget(title)
        titles.addWidget(label('底盘与作业装置 · 仿真 / USB 串口实机', True))
        header.addLayout(titles)
        header.addStretch()
        self.backend_badge = label('等待后端')
        self.backend_badge.setObjectName('badge')
        self.phase_badge = label('已停止')
        self.phase_badge.setObjectName('badge')
        header.addWidget(self.backend_badge)
        header.addWidget(self.phase_badge)
        outer.addLayout(header)
        self.connection_label = label('等待控制管理节点和板卡状态…', True)
        self.connection_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        outer.addWidget(self.connection_label)

        columns = QtWidgets.QHBoxLayout()
        columns.setSpacing(16)
        outer.addLayout(columns, 1)
        # Scroll controls on small monitors; stop buttons remain outside this area.
        controls_scroll = QtWidgets.QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        controls_scroll.setMinimumWidth(485)
        controls_widget = QtWidgets.QWidget()
        controls_scroll.setWidget(controls_widget)
        left = QtWidgets.QVBoxLayout(controls_widget)
        left.setContentsMargins(0, 0, 5, 0)
        left.setSpacing(8)
        columns.addWidget(controls_scroll, 5)

        chassis = QtWidgets.QGroupBox('01  底盘 · 双轮差速')
        grid = QtWidgets.QGridLayout(chassis)
        self.editors['linear_mps'] = spin(-.35, .35, .01, 'm/s', 3)
        self.editors['angular_radps'] = spin(-2, 2, .05, 'rad/s', 3)
        for row, (title, key, hint) in enumerate((
                ('线速度 v', 'linear_mps', '负数后退 / 正数前进'),
                ('角速度 ω', 'angular_radps', '负数右转 / 正数左转'))):
            grid.addWidget(label(title), row, 0)
            grid.addWidget(self.editors[key], row, 1)
            grid.addWidget(label(hint, True), row, 2)
        grid.addWidget(label('范围：±0.35 m/s，±2 rad/s；轮速由下位机分配。', True), 2, 0, 1, 3)
        left.addWidget(chassis)

        implements = QtWidgets.QGroupBox('02  作业装置 · 完整控制快照')
        grid = QtWidgets.QGridLayout(implements)
        for col, title in enumerate(('装置 / 开启', '目标值', '后端能力')):
            grid.addWidget(label(title, True), 0, col)
        for row, field in enumerate(ACTUATORS, 1):
            checkbox = QtWidgets.QCheckBox(field.title)
            editor = spin(field.minimum, field.maximum, field.step, field.unit)
            status = label('等待状态', True)
            self.checks[field.key] = checkbox
            self.editors[field.value_field] = editor
            self.support[field.key] = status
            grid.addWidget(checkbox, row, 0)
            grid.addWidget(editor, row, 1)
            grid.addWidget(status, row, 2)
        grid.addWidget(label('未勾选的装置发送关闭、目标归零。灰色字段表示当前板卡不支持。', True), 6, 0, 1, 3)
        left.addWidget(implements)

        self.draft_status = label('目标均为零')
        left.addWidget(self.draft_status)
        self.start_button = QtWidgets.QPushButton('开始发送 / 使能')
        self.start_button.setObjectName('start')
        self.apply_button = QtWidgets.QPushButton('应用修改')
        self.reset_button = QtWidgets.QPushButton('清零草稿')
        self.disable_button = QtWidgets.QPushButton('撤销使能')
        left.addWidget(label('开始后以 50 Hz 持续发送。编辑不立即生效，需点击“应用修改”。关闭窗口或界面超时会停止发送。', True))
        left.addStretch()

        right = QtWidgets.QVBoxLayout()
        right.setSpacing(10)
        columns.addLayout(right, 6)
        feedback = QtWidgets.QGroupBox('下位机反馈')
        f = QtWidgets.QVBoxLayout(feedback)
        self.summary_label = label('等待状态…')
        self.summary_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        f.addWidget(self.summary_label)
        self.table = QtWidgets.QTableWidget(7, 3)
        self.table.setHorizontalHeaderLabels(['字段', '设定 / 状态', '实际 / 附加反馈'])
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.table.setMinimumHeight(270)
        self.table.setMaximumHeight(290)
        for row, name in enumerate(('底盘 v / ω', '左右驱动轮', '左播撒', '右播撒', '割草刀盘', '刀盘升降', '喷洒装置')):
            for col, value in enumerate((name, '—', '—')):
                self.table.setItem(row, col, QtWidgets.QTableWidgetItem(value))
            self.table.setRowHeight(row, 32)
        f.addWidget(self.table)
        f.addWidget(label('刀盘只反馈油门设定；“已停止”表示软件状态，不能确认机械停转。', True))
        right.addWidget(feedback)

        self.tabs = QtWidgets.QTabWidget()
        self.command_view = QtWidgets.QPlainTextEdit()
        self.command_view.setReadOnly(True)
        self.command_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.event_view = QtWidgets.QPlainTextEdit()
        self.event_view.setReadOnly(True)
        self.tabs.addTab(self.command_view, 'PC 完整指令')
        self.tabs.addTab(self.event_view, '操作记录')
        right.addWidget(self.tabs, 1)
        right.addWidget(label('这里显示控制管理节点的输出。串口组帧由驱动完成；板卡最近命令序号与结果见上方。', True))

        self.notice = label()
        self.notice.setObjectName('notice')
        outer.addWidget(self.notice)
        self.reason_label = label()
        outer.addWidget(self.reason_label)
        bottom = QtWidgets.QHBoxLayout()
        for button in (self.start_button, self.apply_button, self.reset_button, self.disable_button):
            button.setMinimumHeight(43)
            bottom.addWidget(button)
        bottom.addStretch()
        self.stop_button = QtWidgets.QPushButton('■  停止全部')
        self.stop_button.setObjectName('stop')
        self.stop_button.setMinimumSize(190, 49)
        bottom.addWidget(self.stop_button)
        outer.addLayout(bottom)

        self.start_button.clicked.connect(lambda: self.invoke(self.node.start))
        self.apply_button.clicked.connect(lambda: self.invoke(self.node.apply))
        self.reset_button.clicked.connect(self.reset_draft)
        self.disable_button.clicked.connect(lambda: self.node.stop('已请求撤销使能', disable=True))
        self.stop_button.clicked.connect(lambda: self.node.stop())
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self.pump)
        self.timer.start(10)
        self.next_refresh = 0.0
        self.refresh()

    def draft(self, commit=True):
        # Interpret typed text even when the editor has not lost keyboard focus yet.
        if commit:
            for editor in self.editors.values():
                editor.interpretText()
        values = {key: editor.value() for key, editor in self.editors.items()}
        values.update({key + '_on': checkbox.isChecked() for key, checkbox in self.checks.items()})
        return Selection(**values)

    def reset_draft(self):
        for editor in self.editors.values():
            editor.setValue(0)
        for checkbox in self.checks.values():
            checkbox.setChecked(False)
        self.node.note('草稿已清零；持续发送时需应用修改才生效')

    def invoke(self, operation):
        try:
            operation(self.draft())
        except (ValueError, RuntimeError) as error:
            self.node.note(str(error))
        self.refresh()

    def pump(self):
        if not rclpy.ok():
            self.close_ready = True
            self.close()
            return
        # Bound callback work. ROS and Qt share this thread, including the heartbeat.
        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.node.tick()
        if time.monotonic() >= self.next_refresh:
            self.refresh()
            self.next_refresh = time.monotonic() + .100

    def refresh(self):
        node = self.node
        board = node.board
        error = node.health_error()
        live = board is not None and time.monotonic() - node.board_time <= node.timeout
        caps = board.capabilities if live else 0
        simulated = board is not None and board.simulated
        self.backend_badge.setText(('仿真后端' if simulated else '实机后端') if board else '等待后端')
        self.backend_badge.setStyleSheet('background: #d9eee9; color: #166358;' if simulated else 'background: #f6e4d1; color: #7b4814;')
        names = {'idle': '已停止发送', 'arming': '请求使能…', 'waiting': '等待确认…', 'running': '发送中 · 50 Hz'}
        self.phase_badge.setText(names[node.phase])
        self.phase_badge.setStyleSheet('background: #087f8c; color: white;' if node.phase == 'running' else '')
        age = f'{max(0, (time.monotonic() - node.board_time) * 1000):.0f} ms' if board else '—'
        backend = node.backend_label or node.device
        self.connection_label.setText(f'{backend}  |  状态年龄 {age}  |  连接 {board.connection_id if board else "—"}')
        for key in ('linear_mps', 'angular_radps'):
            self.editors[key].setEnabled(bool(caps & 1) and not self.closing)
        for field in ACTUATORS:
            supported = bool(caps & (1 << field.bit))
            if live and not supported:
                self.checks[field.key].setChecked(False)
            self.checks[field.key].setEnabled(supported and not self.closing)
            self.editors[field.value_field].setEnabled(supported and not self.closing)
            self.support[field.key].setText('可控制' if supported else ('不支持' if live else '等待状态'))
        self.start_button.setEnabled(not self.closing and node.phase == 'idle' and not error and node.enable_future is None)
        self.apply_button.setEnabled(not self.closing and node.phase == 'running' and not error)
        self.reset_button.setEnabled(not self.closing)
        draft = self.draft(commit=False).normalized()
        if node.phase == 'running':
            self.draft_status.setText('● 修改尚未应用' if draft != node.active else '● 当前目标已应用')
        else:
            self.draft_status.setText('编辑目标，然后点击“开始发送”')
        connection_error = error
        if error and node.device_status:
            connection_error += '；' + node.device_status
        self.notice.setText(connection_error or (
            ('仿真运行中 · 修改目标后点击“应用修改”' if simulated else '实机运行中 · 正在控制实际装置')
            if node.phase == 'running' else
            ('仿真连接就绪 · 可设置目标并开始发送' if simulated else '实机连接就绪 · 开始发送后会驱动实际装置')))
        self.reason_label.setText(node.reason)
        if board:
            modes = {0: '尚未接管', 1: '运行', 2: '停机 / 禁用'}
            results = {0: '接纳', 1: '帧错误', 2: '参数错误', 3: '能力不支持', 4: '保留值'}
            self.summary_label.setText(
                f'{"实时" if live else "已过期"}  ·  {modes.get(board.mode, str(board.mode))}  ·  能力 0x{board.capabilities:04X}\n'
                f'状态序号 {board.frame_seq}  /  最近命令 {board.last_command_seq}  /  {results.get(board.result, str(board.result))}\n'
                f'故障 0x{board.faults:04X}  ·  指令年龄 {board.command_age_ms} ms  ·  接收错误 {board.rx_error_count}  ·  上电 {board.uptime_ms / 1000:.1f} s')
            c = board.chassis
            on = lambda value: '开' if value else '关'
            values = [
                (f'{c.linear_mps:.3f} m/s', f'{c.angular_radps:.3f} rad/s'),
                (f'{c.left_target_rpm:.1f} / {c.right_target_rpm:.1f} RPM', f'{c.left_actual_rpm:.1f} / {c.right_actual_rpm:.1f} RPM'),
                (f'{on(board.left_spreader.on)} · {board.left_spreader.target_rpm:.1f} RPM', f'{board.left_spreader.actual_rpm:.1f} RPM / 舵机 {board.left_spreader.servo_set_angle_deg:.1f}°'),
                (f'{on(board.right_spreader.on)} · {board.right_spreader.target_rpm:.1f} RPM', f'{board.right_spreader.actual_rpm:.1f} RPM / 舵机 {board.right_spreader.servo_set_angle_deg:.1f}°'),
                (f'{on(board.mower.on)} · {board.mower.throttle_set_percent:.1f} %', '无转速反馈'),
                (f'{on(board.lift.on)} · {board.lift.target_mm:.1f} mm', f'{board.lift.actual_mm:.1f} mm' if board.lift.feedback_valid else '反馈无效'),
                (f'{on(board.sprayer.on)} · {board.sprayer.target_percent:.1f} %', f'{board.sprayer.actual_percent:.1f} %' if board.sprayer.feedback_valid else '反馈无效'),
            ]
            for row, pair in enumerate(values):
                for col, text in enumerate(pair, 1):
                    self.table.item(row, col).setText(text)
        if node.command:
            command = node.command
            age = node.get_clock().now().nanoseconds * 1e-9 - (command.header.stamp.sec + command.header.stamp.nanosec * 1e-9)
            fields = {'enable': command.enable, 'stop': command.stop,
                      'linear_mps': round(command.linear_mps, 3), 'angular_radps': round(command.angular_radps, 3)}
            for field in ACTUATORS:
                fields[field.key] = {'on': getattr(command.implements, field.key + '_on'),
                                     field.value_field: round(getattr(command.implements, field.value_field), 2)}
            lines = [f'  "{key}": {json.dumps(value, ensure_ascii=False)}' for key, value in fields.items()]
            text = f'PC 输出快照 · {"已过期" if age > node.timeout else "实时"}\n' + '{\n' + ',\n'.join(lines) + '\n}'
            if self.command_view.toPlainText() != text:
                scrollbar = self.command_view.verticalScrollBar()
                position = scrollbar.value()
                self.command_view.setPlainText(text)
                scrollbar.setValue(position)
        events = '\n'.join(node.events)
        if self.last_events != events:
            self.event_view.setPlainText(events)
            self.event_view.verticalScrollBar().setValue(self.event_view.verticalScrollBar().maximum())
            self.last_events = events

    def closeEvent(self, event):
        if self.close_ready or (self.node.phase == 'idle' and self.node.enable_future is None):
            self.timer.stop()
            event.accept()
            return
        event.ignore()
        if not self.closing:
            self.closing = True
            self.node.stop('窗口关闭，已请求停止全部')
            self.refresh()
            # Give the asynchronous Stop and any in-flight Enable a chance to settle.
            # Backends also stop independently when publication ends.
            QtCore.QTimer.singleShot(350, self.finish_close)

    def finish_close(self):
        self.close_ready = True
        self.close()


def main(args=None):
    import os
    import signal
    import sys
    from rclpy.signals import SignalHandlerOptions
    from .controller import PanelNode
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY') and os.environ.get('QT_QPA_PLATFORM') not in ('offscreen', 'minimal'):
        print('控制面板需要图形桌面。无显示环境请使用 start_sim.sh --headless。', file=sys.stderr)
        return 1
    # Qt owns shutdown: keep the ROS context alive while Stop is being delivered.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setStyle('Fusion')
    app.setApplicationName('TerraMind')
    app.setFont(QtGui.QFont('Noto Sans CJK SC', 10))
    node = PanelNode()
    window = PanelWindow(node)
    shutdown_requested = False

    def request_shutdown(*_):
        nonlocal shutdown_requested
        if not shutdown_requested:
            shutdown_requested = True
            # A terminal and ros2 launch can both deliver SIGINT. Avoid a
            # reentrant close from inside a ROS callback or Qt closeEvent.
            QtCore.QTimer.singleShot(0, window.close)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    window.show()
    try:
        return app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
