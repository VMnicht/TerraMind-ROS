"""ROS endpoint driven by the GUI event loop: a frozen UI cannot keep motion alive."""
from collections import deque
from datetime import datetime
import time

from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool, Trigger
from diagnostic_msgs.msg import DiagnosticArray
from terramind_interfaces.msg import ImplementCommand, McuState, ControlStatus, ControlCommand
from .controls import ACTUATORS, Selection


class PanelNode(Node):
    def __init__(self):
        super().__init__('terramind_panel')
        self.timeout = float(self.declare_parameter('state_timeout_s', .150).value)
        self.backend_label = str(self.declare_parameter('backend_label', '').value)
        if not .05 <= self.timeout < .250:
            raise ValueError('state_timeout_s must be in [0.05, 0.250)')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('This panel uses the real-time MVP backend')
        self.velocity = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.implements = self.create_publisher(ImplementCommand, 'implements/command', 1)
        self.enable_client = self.create_client(SetBool, 'control/set_enabled')
        self.stop_client = self.create_client(Trigger, 'control/stop')
        self.create_subscription(McuState, 'mcu/state', self._on_board, 1)
        self.create_subscription(ControlStatus, 'control/status', self._on_control, 1)
        self.create_subscription(ControlCommand, 'mcu/command', self._on_command, 1)
        self.create_subscription(DiagnosticArray, 'diagnostics', self._on_diagnostic, 10)
        self.board = self.control = self.command = None
        self.board_time = self.control_time = -1.0
        self.control_stamp = -1.0
        self.device = '等待后端'
        self.device_status = ''
        self.phase = 'idle'
        self.active = Selection()
        self.connection = 0
        self.generation = 0
        self.enable_future = None
        self.enable_stage = 'stop'
        self.enable_generation = 0
        self.stop_futures = []
        self.arming_at = self.arming_stamp = 0.0
        self.last_tick = time.monotonic()
        self.next_send = self.next_graph_check = 0.0
        self.sent_count = 0
        self.reason = '等待连接；启动后保持停机'
        self.events = deque(maxlen=80)
        self.note(self.reason)

    def note(self, text):
        self.reason = text
        self.events.append(f'{datetime.now():%H:%M:%S}  {text}')

    def _source_time(self, header):
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or age < -.020 or age > self.timeout:
            return None
        return time.monotonic() - max(0.0, age)

    def _on_board(self, message):
        received = self._source_time(message.header)
        if received is None:
            return
        if self.phase != 'idle' and message.connection_id != self.connection:
            self.stop('连接已更换，需要重新开始发送')
        self.board, self.board_time = message, received

    def _on_control(self, message):
        received = self._source_time(message.header)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if received is not None and stamp > self.control_stamp:
            self.control, self.control_time, self.control_stamp = message, received, stamp

    def _on_command(self, message):
        if self._source_time(message.header) is not None:
            self.command = message

    def _on_diagnostic(self, message):
        for status in message.status:
            if status.name == 'mcu_serial':
                self.device = status.hardware_id
                self.device_status = status.message
            elif status.name == 'mcu_sim':
                self.device = '二维运动模型'

    def health_error(self):
        now = time.monotonic()
        if self.board is None or now - self.board_time > self.timeout:
            return '未收到及时的板卡状态'
        if self.control is None or now - self.control_time > self.timeout:
            return '未收到及时的控制管理状态'
        if not self.board.link_ready or not self.control.link_ready:
            return '后端尚未就绪'
        if self.board.connection_id != self.control.connection_id:
            return '正在等待新连接同步'
        if self.board.result != 0 or self.board.faults & 3:
            return '板卡报告拒绝命令或故障'
        return ''

    def input_conflict(self):
        if self.count_publishers('cmd_vel') > 1 or self.count_publishers('implements/command') > 1:
            return '检测到其他控制输入，请先关闭其他面板或控制工具'
        if self.count_publishers('mcu/state') != 1 or self.count_publishers('control/status') != 1:
            return '需要唯一的后端和控制管理节点'
        return ''

    def start(self, draft):
        error = self.health_error() or self.input_conflict()
        if error:
            raise ValueError(error)
        if self.phase != 'idle' or self.enable_future is not None:
            raise ValueError('上一个使能请求尚未结束')
        if self.control.enabled:
            raise ValueError('控制已被使能，请先点击停止全部再接管')
        if not self.enable_client.service_is_ready() or not self.stop_client.service_is_ready():
            raise ValueError('使能或停止服务尚未就绪')
        if not self.velocity.get_subscription_count() or not self.implements.get_subscription_count():
            raise ValueError('控制输入尚未连接')
        draft.validate(self.board.capabilities)
        self.active = draft.normalized()
        self.connection = self.board.connection_id
        self.generation += 1
        self.enable_generation = self.generation
        self.phase = 'arming'
        self.arming_at = time.monotonic()
        self.arming_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.next_send = 0.0
        self._publish(Selection())
        # A fresh Stop response acts as a barrier for prior queued Stop requests.
        # Otherwise a delayed Stop after a service outage can overtake a new Enable.
        self.enable_stage = 'stop'
        try:
            self.enable_future = self.stop_client.call_async(Trigger.Request())
        except RuntimeError:
            self.stop('使能请求发送失败')
            raise
        self.note('正在确认停止状态，然后请求使能')

    def apply(self, draft):
        if self.phase != 'running':
            raise ValueError('请先开始发送')
        error = self.health_error()
        if error:
            self.stop(error)
            raise ValueError(error)
        draft.validate(self.board.capabilities)
        self.active = draft.normalized()
        self.note('已应用全部目标，持续发送中')

    def _request_stop(self, disable=False):
        if disable and self.enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self.stop_futures.append(self.enable_client.call_async(request))
        elif self.stop_client.service_is_ready():
            self.stop_futures.append(self.stop_client.call_async(Trigger.Request()))
        self.stop_futures = [future for future in self.stop_futures if not future.done()][-8:]

    def stop(self, reason='已请求停止全部；再次运行需要重新开始', disable=False):
        self.generation += 1
        self.phase = 'idle'
        self.active = Selection()
        self._publish(Selection())
        self._request_stop(disable)
        self.note(reason)

    def _publish(self, selection):
        stamp = self.get_clock().now().to_msg()
        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = 'base_link'
        velocity.twist.linear.x = float(selection.linear_mps)
        velocity.twist.angular.z = float(selection.angular_radps)
        command = ImplementCommand()
        command.header = velocity.header
        for field in ACTUATORS:
            setattr(command, field.key + '_on', getattr(selection, field.key + '_on'))
            setattr(command, field.value_field, float(getattr(selection, field.value_field)))
        self.velocity.publish(velocity)
        self.implements.publish(command)
        self.sent_count += 1

    def tick(self):
        now = time.monotonic()
        gap = now - self.last_tick
        self.last_tick = now
        if self.phase != 'idle' and gap > self.timeout:
            self.stop('界面响应超时，已停止；需要重新开始')
        if self.enable_future is not None and not self.enable_future.done() and now - self.arming_at > 2.0:
            # A disappeared service must not leave the UI permanently unstartable.
            # Cancellation is local only. Stop again; a late server-side Enable
            # cannot receive fresh motion inputs from this now-idle panel.
            client = self.stop_client if self.enable_stage == 'stop' else self.enable_client
            client.remove_pending_request(self.enable_future)
            self.enable_future.cancel()
            self.enable_future = None
            self.stop('使能请求无响应，已停止；连接恢复后可重新开始')
        if self.enable_future is not None and self.enable_future.done():
            future, self.enable_future = self.enable_future, None
            if self.enable_generation != self.generation or self.phase != 'arming':
                # An earlier enable may finish after Stop: settle it with another Stop.
                self._request_stop()
            elif future.exception() is not None or not future.result().success:
                message = str(future.exception()) if future.exception() else future.result().message
                self.stop('使能失败：' + message)
            elif self.enable_stage == 'stop':
                self.enable_stage = 'enable'
                request = SetBool.Request()
                request.data = True
                self.enable_future = self.enable_client.call_async(request)
                self.note('已请求使能，正在等待确认')
            else:
                self.phase = 'waiting'
        if self.phase == 'idle':
            return
        error = self.health_error()
        if not error and self.board.connection_id != self.connection:
            error = '连接已更换'
        if not error and now >= self.next_graph_check:
            error = self.input_conflict()
            self.next_graph_check = now + .25
        if error:
            self.stop(error + '；已停止')
            return
        if self.phase in ('arming', 'waiting'):
            if now - self.arming_at > 1.0:
                self.stop('使能确认超时')
                return
            if self.phase == 'waiting' and self.control.enabled and self.control_stamp >= self.arming_stamp:
                self.phase = 'running'
                self.note('持续发送中 · 50 Hz')
        elif not self.control.enabled:
            self.stop('控制管理已撤销使能：' + self.control.reason)
            return
        try:
            self.active.validate(self.board.capabilities)
        except ValueError as error:
            self.stop(str(error))
            return
        if now >= self.next_send:
            self._publish(self.active if self.phase == 'running' else Selection())
            # Keep the 20 ms phase across Qt timer jitter; never replay missed ticks.
            self.next_send = self.next_send + .020 if now - self.next_send < .020 else now + .020
