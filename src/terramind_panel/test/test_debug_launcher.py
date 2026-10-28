"""总面板回归：进程组清理、命令引用、任务互斥和构建后启动。"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
from PyQt5 import QtCore, QtWidgets
import pytest
from terramind_panel import debug_runner
from terramind_panel.debug_panel import DebugWindow, Task, backend_task, shell_command


@pytest.fixture(scope='module')
def app():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield application


@pytest.fixture
def ros_available(monkeypatch):
    # 这些单元测试只验证排队/互斥，不执行 ROS。环境检查与进程联调分开。
    exists = Path.exists
    monkeypatch.setattr(Path, 'exists', lambda path:
                        True if str(path) == '/opt/ros/humble/setup.bash' else exists(path))


def wait(app, predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('condition timed out')


def alive(pid):
    path = Path(f'/proc/{pid}/stat')
    try:
        # 已退出的孤儿进程可能仍等待容器 init 回收，不能误判为仍在运行。
        return path.read_text().split(') ', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


@pytest.mark.parametrize('disconnect', [False, True])
def test_supervisor_stops_descendants(tmp_path, disconnect):
    marker = tmp_path / 'child'
    code = ('import subprocess,time,pathlib; '
            'p=subprocess.Popen(["/usr/bin/python3", "-c", "import time; time.sleep(60)"]); '
            f'pathlib.Path({str(marker)!r}).write_text(str(p.pid)); time.sleep(60)')
    process = subprocess.Popen([sys.executable, debug_runner.__file__, sys.executable, '-c', code],
                               stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    child = None
    try:
        deadline = time.monotonic() + 3
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        child = int(marker.read_text())
        if not disconnect:
            process.stdin.write(b'stop\n')
            process.stdin.flush()
        else:
            process.stdin.close()
        assert process.wait(timeout=15) == 130
        assert not alive(child)
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=15)
        if child and alive(child):
            os.kill(child, signal.SIGKILL)


def test_supervisor_cleans_stubborn_orphan_on_parent_exit(tmp_path):
    marker = tmp_path / 'orphan'
    child_code = ('import signal,time,pathlib,os; '
                  'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                  'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                  f'pathlib.Path({str(marker)!r}).write_text(str(os.getpid())); time.sleep(60)')
    code = ('import subprocess,time,pathlib; '
            f'subprocess.Popen(["/usr/bin/python3", "-c", {child_code!r}]); '
            f'p=pathlib.Path({str(marker)!r}); '
            '\nwhile not p.exists(): time.sleep(.01)\n')
    process = subprocess.Popen([sys.executable, debug_runner.__file__, sys.executable, '-c', code],
                               stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        assert process.wait(timeout=16) == 0
        assert not alive(int(marker.read_text()))
    finally:
        process.stdin.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=15)


def test_shell_arguments_cannot_execute_user_input(tmp_path):
    workspace = tmp_path / 'space and $special'
    (workspace / 'install').mkdir(parents=True)
    (workspace / 'install/local_setup.bash').write_text(':\n')
    marker = tmp_path / 'INJECTED'
    device = f'/dev/a; touch {marker}; $(touch {marker})'
    task = backend_task('real', device=device)
    command = shell_command(workspace, task)
    # 用只打印参数的程序替代 ROS 命令，再执行实际生成的 shell，
    # 验证用户输入始终保持参数边界，不能成为额外命令。
    command[2] = command[2].replace('source /opt/ros/humble/setup.bash',
                                   "ros2() { printf '%s\\n' \"$@\"; }; export -f ros2")
    command[2] = command[2].replace('exec ros2 ', 'ros2 ')
    output = subprocess.check_output(command, text=True)
    assert 'device:=' + device in output.splitlines()
    assert not marker.exists()


def test_window_conflicts_and_backend_options(app, tmp_path, monkeypatch, ros_available):
    window = DebugWindow(tmp_path)
    launched = []
    monkeypatch.setattr(window, 'artifacts_ready', lambda: True)
    monkeypatch.setattr(window, 'spawn', launched.append)
    assert window.capabilities.currentData() == 47
    assert window.capabilities.itemData(2) == 15  # Legacy board remains selectable.
    window.mode.setCurrentIndex(1)
    window.capabilities.setCurrentIndex(1)
    window.launch_button.click()
    assert launched[-1].command[-1] == 'capabilities:=63'
    assert 'panel:=true' in launched[-1].command
    window.mode.setCurrentIndex(2)
    assert window.device.isEnabled() and not window.capabilities.isEnabled()
    assert not window.rviz.isEnabled()
    window.device.setText('ttyUSB0')
    window.launch_button.click()
    assert len(launched) == 1
    window.close()


def test_standalone_panel_uses_workspace_limits(app, tmp_path, monkeypatch):
    workspace = tmp_path / 'Linux workspace'
    window = DebugWindow(workspace)
    launched = []
    monkeypatch.setattr(window, 'request', launched.append)
    window.launch_panel()
    command = launched[-1].command
    assert command[command.index('--params-file') + 1] == str(
        workspace / 'src/terramind_bringup/config/control.yaml')
    assert launched[-1].panel
    window.close()


def test_auto_build_only_launches_after_success(app, tmp_path, monkeypatch, ros_available):
    window = DebugWindow(tmp_path)
    launched = []
    monkeypatch.setattr(window, 'spawn', launched.append)
    window.launch_backend()
    assert launched[-1].key == 'build' and window.pending.key == 'backend'
    class Done:
        task = Task('build', 'build', [])
        state = '失败 (1)'
    window.job_finished(Done())
    assert window.pending is None and len(launched) == 1
    window.launch_backend()
    Done.state = '已完成'
    window.job_finished(Done())
    assert launched[-1].key == 'backend'
    window.close()


def test_navigation_launch_options(app, tmp_path, monkeypatch):
    config = tmp_path / 'navigation parameters.yaml'
    config.write_text('navigation_serial_node: {}\n')
    window = DebugWindow(tmp_path)
    launched = []
    monkeypatch.setattr(window, 'request', launched.append)
    window.mode.setCurrentIndex(2)
    window.navigation.setChecked(True)
    window.navigation_config.setText(str(config))
    window.navigation_device.setText('/dev/navigation-test')
    assert window.rviz.isEnabled()
    window.launch_backend()
    command = launched[-1].command
    assert 'navigation:=true' in command
    assert 'navigation_config:=' + str(config) in command
    assert 'navigation_device:=/dev/navigation-test' in command
    assert 'navigation_rviz:=true' in command
    window.navigation_device.setText('auto')
    window.launch_backend()
    assert len(launched) == 2
    assert 'navigation_device:=auto' in launched[-1].command
    window.navigation_device.setText('')
    window.launch_backend()
    assert 'navigation_device:=' in launched[-1].command
    window.navigation_device.setText('not-a-device')
    window.launch_backend()
    assert len(launched) == 3
    window.close()


def test_navigation_editor_preserves_file_on_invalid_yaml(app, tmp_path, monkeypatch):
    config = tmp_path / 'navigation.yaml'
    original = 'navigation_serial_node:\n  ros__parameters:\n    origin_height_m: 20.0\n'
    config.write_text(original)
    window = DebugWindow(tmp_path)
    window.navigation_config.setText(str(config))
    contents = ['[invalid', original.replace('20.0', '21.0')]
    errors = []
    monkeypatch.setattr(QtWidgets.QMessageBox, 'warning', lambda *args: errors.append(args[-1]))

    def interact(dialog):
        dialog.findChild(QtWidgets.QPlainTextEdit).setPlainText(contents.pop(0))
        dialog.findChild(QtWidgets.QDialogButtonBox).accepted.emit()
        return 0

    monkeypatch.setattr(QtWidgets.QDialog, 'exec_', interact)
    window.edit_navigation_config()
    assert errors and config.read_text() == original
    window.edit_navigation_config()
    assert config.read_text() == original.replace('20.0', '21.0')
    window.close()


def test_ui_logs_failure_conflicts_and_close_cleanup(app, tmp_path, monkeypatch):
    import terramind_panel.debug_panel as panel
    monkeypatch.setattr(panel, 'shell_command', lambda workspace, task: task.command)
    window = DebugWindow(tmp_path)
    window.spawn(Task('backend', 'fake backend',
                      [sys.executable, '-u', '-c', 'import time; print("ready"); time.sleep(60)'], panel=True))
    wait(app, lambda: 'ready\n' in window.jobs[-1].buffer)
    assert not window.launch_button.isEnabled()
    assert not window.panel_button.isEnabled()
    assert not window.build_button.isEnabled()
    window.request(Task('backend', 'duplicate', []))
    assert len(window.jobs) == 1
    window.show()
    window.close()
    wait(app, lambda: not window.active(), timeout=15)
    wait(app, lambda: not window.isVisible())
    assert window.jobs[-1].state == '已停止'
    assert 'ready' in window.jobs[-1].path.read_text()
    failed = DebugWindow(tmp_path)
    failed.spawn(Task('test', 'failure', [sys.executable, '-c', 'raise SystemExit(7)']))
    wait(app, lambda: not failed.active())
    assert failed.jobs[-1].state == '失败 (7)'
    failed.close()
