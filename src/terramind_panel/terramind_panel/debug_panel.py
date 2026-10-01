"""Desktop debug launcher. ROS commands run in supervised child processes."""
import argparse
import codecs
from dataclasses import dataclass
from datetime import datetime
import fcntl
import os
from pathlib import Path
import shlex
import signal
import sys

from PyQt5 import QtCore, QtGui, QtWidgets
from .controls import CURRENT_CAPABILITIES, LEGACY_CAPABILITIES, ALL_CAPABILITIES


BUILD = ['colcon', 'build', '--symlink-install', '--parallel-workers', '2',
         '--cmake-args', '-DCMAKE_BUILD_TYPE=RelWithDebInfo']
TESTS = [
    ('单元测试 · 全部包', 'unit', []),
    ('串口协议回环', 'terramind_sim', ['serial_loopback_test.py']),
    ('串口自动发现', 'terramind_sim', ['serial_discovery_test.py']),
    ('ROS 控制链路 · 二维仿真', 'terramind_sim', ['ros_smoke_test.py', '--mode', 'sim']),
    ('ROS 控制链路 · 虚拟串口', 'terramind_sim', ['ros_smoke_test.py', '--mode', 'serial']),
    ('控制面板 · 二维仿真 / 全能力', 'terramind_panel', ['panel_integration_test.py', '--mode', 'sim']),
    ('控制面板 · 虚拟串口 / 全能力', 'terramind_panel', ['panel_integration_test.py', '--mode', 'serial']),
    ('控制面板 · 当前板卡 / 含喷洒', 'terramind_panel', ['panel_integration_test.py', '--mode', 'sim', '--capabilities', str(CURRENT_CAPABILITIES)]),
    ('控制面板 · 虚拟串口 / 含喷洒', 'terramind_panel', ['panel_integration_test.py', '--mode', 'serial', '--capabilities', str(CURRENT_CAPABILITIES)]),
    ('控制面板 · 旧板卡 / 不含喷洒', 'terramind_panel', ['panel_integration_test.py', '--mode', 'sim', '--capabilities', str(LEGACY_CAPABILITIES)]),
    ('调试总面板 · 启动与停止联调', 'terramind_panel', ['debug_launcher_integration_test.py']),
]
STYLE = '''
QWidget { font-family: "Noto Sans CJK SC"; font-size: 13px; color: #203743; }
QMainWindow { background: #edf2f5; }
QGroupBox { background: white; border: 1px solid #d6e1e6; border-radius: 8px;
 margin-top: 15px; padding: 16px 13px 10px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
QLabel#title { font-size: 25px; font-weight: 700; }
QLabel#muted { color: #617783; }
QPushButton { background: #f6f9fb; border: 1px solid #bdcdd5; border-radius: 5px;
 padding: 6px 14px; min-height: 23px; }
QPushButton:hover { background: #e3eff3; }
QPushButton#primary { color: white; background: #087f8c; border-color: #087f8c; }
QPushButton#stop { color: #a92b3c; background: #fff2f3; border-color: #e6bac1; }
QPushButton:disabled { color: #8d9ca5; background: #e6ecef; border-color: #d5dfe4; }
QComboBox, QLineEdit, QSpinBox { padding: 5px; min-height: 22px;
 background: white; border: 1px solid #becdd5; border-radius: 4px; }
QTableWidget { background: white; border: 1px solid #d6e1e6; gridline-color: #eaf0f3; }
QHeaderView::section { background: #e7eef2; padding: 7px; border: none; }
QPlainTextEdit { background: #142630; color: #d9e7ee; border-radius: 6px;
 padding: 8px; font-family: "DejaVu Sans Mono"; font-size: 12px; }
'''


@dataclass
class Task:
    key: str
    title: str
    command: list
    overlay: bool = True
    panel: bool = False


def backend_task(mode, capabilities=CURRENT_CAPABILITIES, rviz=True, panel=True, device='auto'):
    arguments = [f'panel:={str(panel).lower()}']
    if mode == 'real':
        device = device.strip() or 'auto'
        if device != 'auto' and not device.startswith('/dev/'):
            raise ValueError('实机串口请填写 auto 或 /dev/ 下的设备路径')
        arguments.append('device:=' + device)
    else:
        arguments.append(f'capabilities:={capabilities}')
        if mode == 'sim':
            arguments.append(f'rviz:={str(rviz).lower()}')
    title = {'sim': '二维仿真', 'serial_test': '虚拟串口联调', 'real': '真实控制板'}[mode]
    return Task('backend', title, ['ros2', 'launch', 'terramind_bringup', mode + '.launch.py', *arguments], panel=panel)


def shell_command(workspace, task):
    # All user-controlled paths and arguments are shell-quoted, never interpolated as code.
    parts = ['set -e', 'source /opt/ros/humble/setup.bash']
    if task.overlay:
        parts.append('source ' + shlex.quote(str(workspace / 'install/local_setup.bash')))
    if task.key == 'test' and task.command == ['unit']:
        parts.extend(['colcon test --event-handlers console_direct+', 'colcon test-result --verbose'])
    else:
        parts.append('exec ' + shlex.join(task.command))
    return ['bash', '-c', '\n'.join(parts)]


class Job(QtCore.QObject):
    changed = QtCore.pyqtSignal()
    completed = QtCore.pyqtSignal(object)

    def __init__(self, task, workspace, domain, parent=None):
        super().__init__(parent)
        self.task = task
        self.state = '启动中'
        self.active = True
        self.stopping = False
        self.decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        self.buffer = ''
        self.log_error = False
        directory = workspace / 'log/debug'
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f'{datetime.now():%Y%m%d-%H%M%S-%f}-{task.key}.log'
        self.logfile = self.path.open('w', encoding='utf-8')
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.setWorkingDirectory(str(workspace))
        env = QtCore.QProcessEnvironment.systemEnvironment()
        env.insert('ROS_DOMAIN_ID', str(domain))
        env.insert('ROS_LOG_DIR', str(workspace / 'log/ros'))
        env.insert('PYTHONUNBUFFERED', '1')
        env.insert('MAKEFLAGS', '-j2 -l2')
        self.process.setProcessEnvironment(env)
        self.process.readyReadStandardOutput.connect(self.read)
        self.process.started.connect(self.started)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.error)
        command = shell_command(workspace, task)
        self.append('$ ' + shlex.join(task.command) + '\n')
        runner = Path(__file__).with_name('debug_runner.py')
        self.process.start('/usr/bin/python3', [str(runner), *command])

    def append(self, value):
        self.buffer = (self.buffer + value)[-160000:]
        if not self.log_error:
            try:
                self.logfile.write(value)
                self.logfile.flush()
            except OSError as error:
                self.log_error = True
                self.buffer += f'\n日志文件写入失败：{error}\n'
        self.changed.emit()

    def read(self):
        self.append(self.decoder.decode(bytes(self.process.readAllStandardOutput())))

    def started(self):
        if self.stopping:
            self.process.write(b'stop\n')
        else:
            self.state = '运行中'
        self.changed.emit()

    def stop(self):
        if self.active and not self.stopping:
            self.stopping = True
            self.state = '正在停止'
            self.append('\n正在停止本任务及子进程…\n')
            self.process.write(b'stop\n')

    def error(self, error):
        self.append('\n进程错误：' + self.process.errorString() + '\n')
        if error == QtCore.QProcess.FailedToStart:
            self.finished(-1, QtCore.QProcess.CrashExit)

    def finished(self, code, exit_status):
        if not self.active:
            return
        self.read()
        self.append(self.decoder.decode(b'', final=True))
        self.active = False
        self.state = ('已停止' if self.stopping else
                      ('已完成' if code == 0 and exit_status == QtCore.QProcess.NormalExit else f'失败 ({code})'))
        self.append(f'\n{self.state} · 退出码 {code}\n')
        self.logfile.close()
        self.changed.emit()
        self.completed.emit(self)


class DebugWindow(QtWidgets.QMainWindow):
    def __init__(self, workspace):
        super().__init__()
        self.workspace = workspace
        self.jobs = []
        self.pending = None
        self.closing = False
        self.setWindowTitle('TerraMind · 调试总面板')
        self.resize(1220, 880)
        self.setStyleSheet(STYLE)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(22, 16, 22, 16)
        title = QtWidgets.QLabel('TerraMind  /  调试总面板')
        title.setObjectName('title')
        layout.addWidget(title)
        subtitle = QtWidgets.QLabel('选择功能即可启动 · 自动加载 ROS 环境 · 统一管理程序与日志')
        subtitle.setObjectName('muted')
        layout.addWidget(subtitle)
        location = QtWidgets.QLabel('工作区：' + str(workspace))
        location.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(location)
        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)
        backend = QtWidgets.QGroupBox('01  启动调试环境')
        top.addWidget(backend, 1)
        form = QtWidgets.QFormLayout(backend)
        self.mode = QtWidgets.QComboBox()
        for name, key in [('二维运动仿真', 'sim'), ('虚拟串口联调（PTY）', 'serial_test'), ('真实控制板（USB 串口）', 'real')]:
            self.mode.addItem(name, key)
        form.addRow('后端', self.mode)
        self.device = QtWidgets.QLineEdit('auto')
        self.device.setPlaceholderText('auto 或 /dev/serial/by-id/…')
        form.addRow('实机端口', self.device)
        self.capabilities = QtWidgets.QComboBox()
        self.capabilities.addItem('当前板卡 · 47（含喷洒，不含升降）', CURRENT_CAPABILITIES)
        self.capabilities.addItem('全部装置 · 63（仅模拟后端）', ALL_CAPABILITIES)
        self.capabilities.addItem('旧板卡 · 15（不含升降 / 喷洒）', LEGACY_CAPABILITIES)
        form.addRow('模拟能力', self.capabilities)
        self.domain = QtWidgets.QSpinBox()
        self.domain.setRange(0, 232)
        try:
            self.domain.setValue(int(os.environ.get('ROS_DOMAIN_ID', '0')))
        except ValueError:
            self.domain.setValue(0)
        form.addRow('ROS 域', self.domain)
        options = QtWidgets.QHBoxLayout()
        self.panel = QtWidgets.QCheckBox('同时打开控制面板')
        self.panel.setChecked(True)
        self.rviz = QtWidgets.QCheckBox('打开 RViz')
        self.rviz.setChecked(True)
        options.addWidget(self.panel)
        options.addWidget(self.rviz)
        form.addRow(options)
        self.backend_hint = QtWidgets.QLabel()
        self.backend_hint.setWordWrap(True)
        form.addRow(self.backend_hint)
        row = QtWidgets.QHBoxLayout()
        self.launch_button = self.button('启动所选后端', self.launch_backend, 'primary')
        self.panel_button = self.button('单独打开控制面板', self.launch_panel)
        row.addWidget(self.launch_button)
        row.addWidget(self.panel_button)
        form.addRow(row)
        utilities = QtWidgets.QGroupBox('02  构建、测试与状态查看')
        top.addWidget(utilities, 1)
        utility_layout = QtWidgets.QVBoxLayout(utilities)
        build_row = QtWidgets.QHBoxLayout()
        self.build_button = self.button('编译 / 更新工作区', self.build)
        build_row.addWidget(self.build_button)
        build_row.addWidget(self.button('打开 README', lambda: self.open_path(workspace / 'README.md')))
        utility_layout.addLayout(build_row)
        self.test_choice = QtWidgets.QComboBox()
        for title, _, _ in TESTS:
            self.test_choice.addItem(title)
        utility_layout.addWidget(self.test_choice)
        self.test_button = self.button('运行所选测试', self.run_test)
        utility_layout.addWidget(self.test_button)
        hint = QtWidgets.QLabel('进程测试自动使用隔离 ROS 域和模拟后端。\n切换后端或构建前，请先停止正在运行的程序。')
        hint.setObjectName('muted')
        hint.setWordWrap(True)
        utility_layout.addWidget(hint)
        topics = QtWidgets.QHBoxLayout()
        self.topic = QtWidgets.QComboBox()
        self.topic.addItems(['/mcu/state', '/control/status', '/mcu/command', '/diagnostics', '/sim/ground_truth'])
        topics.addWidget(self.topic, 1)
        self.observe_button = self.button('查看话题', self.observe)
        topics.addWidget(self.observe_button)
        utility_layout.addLayout(topics)
        utility_layout.addStretch()
        self.notice = QtWidgets.QLabel('启动程序后仍保持未使能；运动与作业目标请在控制面板中操作。')
        self.notice.setWordWrap(True)
        layout.addWidget(self.notice)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        layout.addWidget(splitter, 1)
        task_area = QtWidgets.QWidget()
        task_layout = QtWidgets.QVBoxLayout(task_area)
        task_layout.setContentsMargins(0, 0, 0, 0)
        task_header = QtWidgets.QHBoxLayout()
        task_header.addWidget(QtWidgets.QLabel('03  程序与运行状态'))
        task_header.addStretch()
        self.stop_button = self.button('停止所选程序', self.stop_selected, 'stop')
        self.stop_all_button = self.button('停止全部程序', self.stop_all, 'stop')
        task_header.addWidget(self.stop_button)
        task_header.addWidget(self.stop_all_button)
        task_layout.addLayout(task_header)
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(['任务', '状态', '日志文件'])
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self.show_log)
        task_layout.addWidget(self.table)
        splitter.addWidget(task_area)
        log_area = QtWidgets.QWidget()
        log_layout = QtWidgets.QVBoxLayout(log_area)
        log_layout.setContentsMargins(0, 0, 0, 0)
        log_header = QtWidgets.QHBoxLayout()
        log_header.addWidget(QtWidgets.QLabel('运行输出'))
        log_header.addStretch()
        log_header.addWidget(self.button('打开日志目录', lambda: self.open_path(workspace / 'log/debug')))
        log_layout.addLayout(log_header)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(2000)
        log_layout.addWidget(self.output)
        splitter.addWidget(log_area)
        splitter.setSizes([190, 260])
        self.mode.currentIndexChanged.connect(self.update_controls)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.show_log)
        self.timer.start(200)
        self.rendered = None
        self.update_controls()

    @staticmethod
    def button(title, action, name=''):
        result = QtWidgets.QPushButton(title)
        result.setObjectName(name)
        result.clicked.connect(action)
        return result

    def open_path(self, path):
        if path.suffix == '':
            path.mkdir(parents=True, exist_ok=True)
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def active(self, key=None):
        return [job for job in self.jobs if job.active and (key is None or job.task.key == key)]

    def update_controls(self):
        running = self.active()
        building = bool(self.active('build'))
        reserved = building or self.pending is not None or self.closing
        backend = self.active('backend')
        panel = self.active('panel')
        real = self.mode.currentData() == 'real'
        self.device.setEnabled(not reserved and not backend and real)
        self.capabilities.setEnabled(not reserved and not backend and not real)
        self.rviz.setEnabled(not reserved and not backend and self.mode.currentData() == 'sim')
        self.mode.setEnabled(not reserved and not backend)
        self.panel.setEnabled(not reserved and not backend)
        self.domain.setEnabled(not running and not reserved)
        self.launch_button.setEnabled(not reserved and not backend and not panel and not self.active('test'))
        self.panel_button.setEnabled(not reserved and not panel and not self.active('test') and not any(j.task.panel for j in backend))
        self.build_button.setEnabled(not reserved and not running)
        self.test_button.setEnabled(not reserved and not running)
        self.observe_button.setEnabled(not reserved and not self.active('test'))
        self.stop_all_button.setEnabled(bool(running) or self.pending is not None)
        row = self.table.currentRow()
        self.stop_button.setEnabled(0 <= row < len(self.jobs) and self.jobs[row].active)
        descriptions = {
            'sim': '二维运动模型 + 可选 RViz；无需硬件。',
            'serial_test': '真实串口驱动连接虚拟控制板；无需 USB 设备。',
            'real': '将打开真实串口并发送未使能零指令握手。auto 自动识别唯一控制板。',
        }
        self.backend_hint.setText(descriptions[self.mode.currentData()])

    def artifacts_ready(self):
        required = [
            'install/local_setup.bash',
            'install/terramind_control/lib/terramind_control/control_manager_node',
            'install/terramind_mcu/lib/terramind_mcu/mcu_serial_node',
            'install/terramind_sim/lib/terramind_sim/sim_interface_node',
            'install/terramind_sim/lib/terramind_sim/serial_board_emulator',
            'install/terramind_panel/lib/terramind_panel/control_panel',
            'install/terramind_bringup/share/terramind_bringup/launch/sim.launch.py',
        ]
        return all((self.workspace / path).exists() for path in required)

    def request(self, task):
        if self.closing or self.pending is not None or self.active(task.key) or self.active('build'):
            self.notice.setText('该功能正在运行，或正在等待构建；请先停止现有任务。')
            return
        if ((task.key in ('build', 'test') and self.active()) or
                (task.key != 'test' and self.active('test')) or
                (task.key == 'backend' and self.active('panel')) or
                (task.key == 'panel' and any(j.task.panel for j in self.active('backend')))):
            self.notice.setText('现有任务与所选功能冲突，请先停止对应程序。')
            return
        if not Path('/opt/ros/humble/setup.bash').exists():
            self.notice.setText('未找到 ROS 2 Humble，请先按 README 安装依赖。')
            return
        if task.overlay and not self.artifacts_ready():
            if self.active():
                self.notice.setText('缺少构建产物；请先停止所有程序，再启动以自动编译。')
                return
            self.pending = task
            self.notice.setText('缺少构建产物，正在编译；成功后自动启动：' + task.title)
            self.spawn(Task('build', '自动编译工作区', BUILD, overlay=False))
        else:
            self.spawn(task)

    def spawn(self, task):
        try:
            job = Job(task, self.workspace, self.domain.value(), self)
        except OSError as error:
            self.pending = None
            self.notice.setText('无法启动任务：' + str(error))
            self.update_controls()
            return
        self.jobs.append(job)
        row = self.table.rowCount()
        self.table.insertRow(row)
        for column, text in enumerate([task.title, job.state, job.path.name]):
            self.table.setItem(row, column, QtWidgets.QTableWidgetItem(text))
        job.changed.connect(lambda: self.job_changed(job))
        job.completed.connect(self.job_finished)
        self.table.selectRow(row)
        self.update_controls()

    def job_changed(self, job):
        self.table.item(self.jobs.index(job), 1).setText(job.state)
        self.update_controls()

    def job_finished(self, job):
        if job.task.key == 'build' and self.pending is not None:
            pending, self.pending = self.pending, None
            if job.state == '已完成' and not self.closing:
                self.notice.setText('编译成功，正在启动：' + pending.title)
                self.spawn(pending)
            else:
                self.notice.setText('编译未完成，已取消后续启动。请查看日志。')
        self.update_controls()
        if self.closing and not self.active():
            QtCore.QTimer.singleShot(0, self.close)

    def launch_backend(self):
        try:
            task = backend_task(self.mode.currentData(), self.capabilities.currentData(),
                                self.rviz.isChecked(), self.panel.isChecked(), self.device.text())
        except ValueError as error:
            self.notice.setText(str(error))
            return
        self.request(task)

    def launch_panel(self):
        self.request(Task('panel', '控制面板', ['ros2', 'run', 'terramind_panel', 'control_panel'], panel=True))

    def build(self):
        self.request(Task('build', '编译工作区', BUILD, overlay=False))

    def run_test(self):
        title, package, args = TESTS[self.test_choice.currentIndex()]
        command = ['unit'] if package == 'unit' else ['ros2', 'run', package, *args]
        self.request(Task('test', title, command))

    def observe(self):
        topic = self.topic.currentText()
        self.request(Task('topic-' + topic.replace('/', '-'), '话题 ' + topic, ['ros2', 'topic', 'echo', topic]))

    def stop_selected(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.jobs):
            if self.jobs[row].task.key == 'build':
                self.pending = None
            self.jobs[row].stop()

    def stop_all(self):
        self.pending = None
        for job in self.active():
            job.stop()
        self.update_controls()

    def show_log(self):
        row = self.table.currentRow()
        self.stop_button.setEnabled(0 <= row < len(self.jobs) and self.jobs[row].active)
        if row < 0 or row >= len(self.jobs):
            return
        job = self.jobs[row]
        snapshot = (row, job.buffer)
        if snapshot != self.rendered:
            bar = self.output.verticalScrollBar()
            follow = self.rendered is None or self.rendered[0] != row or bar.value() >= bar.maximum() - 3
            previous = bar.value()
            self.output.setPlainText(job.buffer)
            bar.setValue(bar.maximum() if follow else previous)
            self.rendered = snapshot

    def closeEvent(self, event):
        if self.active():
            self.closing = True
            self.notice.setText('正在停止全部子进程，完成后自动关闭…')
            self.stop_all()
            event.ignore()
        else:
            event.accept()


def find_workspace():
    for candidate in [Path.cwd(), *Path(__file__).resolve().parents]:
        if (candidate / 'src/terramind_bringup').is_dir():
            return candidate
    return None


def main():
    parser = argparse.ArgumentParser(description='TerraMind 调试总面板')
    parser.add_argument('--workspace', type=Path, default=find_workspace())
    args = parser.parse_args()
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setStyle('Fusion')
    if args.workspace is None or not (args.workspace / 'src/terramind_bringup').is_dir():
        QtWidgets.QMessageBox.critical(None, '找不到工作区', '请通过工程中的 start_debug.sh 启动，或指定 --workspace。')
        return 1
    workspace = args.workspace.resolve()
    directory = workspace / 'log/debug'
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory / 'launcher.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        QtWidgets.QMessageBox.information(None, '总面板已打开', '该工作区已有一个调试总面板，请使用现有窗口。')
        return 0
    window = DebugWindow(workspace)
    window.show()
    # Let terminal/desktop termination follow the same asynchronous cleanup path.
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
