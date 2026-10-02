"""监督一个独立进程组；调试总面板退出或断开管道时也清理子进程。"""
import os
import selectors
import signal
import subprocess
import sys
import time


def run(command):
    # 新建会话/进程组，以便同时停止 ros2 launch 和它创建的节点。
    process = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL)
    stopping = False

    def request_stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    selector = selectors.DefaultSelector()
    selector.register(sys.stdin, selectors.EVENT_READ)
    deadline = None
    stage = 0
    cancelled = False

    def send(sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def group_alive():
        try:
            os.killpg(process.pid, 0)
            return True
        except ProcessLookupError:
            return False

    try:
        while True:
            if selector.select(.05):
                os.read(sys.stdin.fileno(), 4096)
                stopping = True  # 收到停止消息或管道 EOF，都表示应停止被监督任务。
                selector.unregister(sys.stdin)
            code = process.poll()
            if deadline is None and (stopping or code is not None):
                cancelled = stopping
                send(signal.SIGINT)
                deadline = time.monotonic() + 8
            if deadline is not None:
                if code is not None and not group_alive():
                    return 130 if cancelled else code
                if time.monotonic() >= deadline:
                    # 先 SIGINT 留给 ROS 正常退出，再逐级升级至 TERM/KILL。
                    # 父进程退出后仍检查组内后代，避免遗留后台节点。
                    if stage == 0:
                        send(signal.SIGTERM)
                        deadline = time.monotonic() + 3
                    elif stage == 1:
                        send(signal.SIGKILL)
                        deadline = time.monotonic() + .5
                    else:
                        return 130 if cancelled else (code if code is not None else 1)
                    stage += 1
    finally:
        send(signal.SIGKILL)
        process.wait()
        selector.close()


if __name__ == '__main__':
    try:
        sys.exit(run(sys.argv[1:]))
    except Exception as error:
        print(f'启动失败：{error}', file=sys.stderr, flush=True)
        sys.exit(1)
