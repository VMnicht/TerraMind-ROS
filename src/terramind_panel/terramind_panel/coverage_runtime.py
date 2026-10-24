"""区域作业接入：定位门控、建区快照及现有唯一控制发布者的自动目标。"""
import math
import time
from collections import deque
from dataclasses import replace
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from .coverage import Pose2, plan_rectangle, wrap
from .coverage_tracking import CoverageTracker
from .controls import Selection


class CoverageRuntime:
    def __init__(self, node):
        self.node = node
        self.source = node.declare_parameter('coverage_pose_source', 'navigation').value
        if self.source not in ('navigation', 'simulation'):
            raise ValueError('coverage_pose_source must be navigation or simulation')
        self.samples = {}
        self.pose_stamps = {}
        self.histories = {}
        self.position_tolerance = float(node.declare_parameter('coverage_position_tolerance_m', .03).value)
        self.yaw_tolerance = float(node.declare_parameter('coverage_yaw_tolerance_rad', .035).value)
        if not (0 < self.position_tolerance <= .2 and 0 < self.yaw_tolerance <= .2):
            raise ValueError('覆盖任务到位容差超出范围')
        self.anchor = self.identity = self.plan = self.tracker = None
        self.tools = Selection()
        self.message = '停稳后以当前位置建区'
        node.create_subscription(PoseStamped, 'navigation/pose', self.on_navigation, qos_profile_sensor_data)
        node.create_subscription(Odometry, 'sim/ground_truth', self.on_simulation, 1)

    def store(self, source, pose, header, identity):
        stamp = self.node._source_time(header)
        q = pose.orientation
        values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
        if stamp is None or not all(math.isfinite(v) for v in values):
            return
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01:
            return
        p = Pose2(pose.position.x, pose.position.y,
                  math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        old = self.samples.get(source)
        ros_stamp = header.stamp.sec*1000000000+header.stamp.nanosec
        # 用原始消息时间去重，避免 ROS/steady 换算的微小抖动使旧位姿续期。
        if old and identity == old[2] and ros_stamp <= self.pose_stamps[source]:
            return
        self.pose_stamps[source] = ros_stamp
        history = self.histories.setdefault(source, deque(maxlen=300))
        if not old or old[2] != identity:
            history.clear()
        history.append((stamp,p))
        while len(history)>1 and stamp-history[0][0] > .7:
            history.popleft()
        self.samples[source] = (p, stamp, identity)

    def nav_identity(self):
        s = self.node.navigation
        if s is None:
            return None
        return (s.header.frame_id, s.session_id, s.origin_latitude_rad,
                s.origin_longitude_rad, s.origin_height_m, s.pose_child_frame)

    def on_navigation(self, message):
        identity = self.nav_identity()
        if identity and message.header.frame_id == identity[0]:
            self.store('navigation', message.pose, message.header, identity)

    def on_simulation(self, message):
        if message.child_frame_id == 'base_link' and self.node.board is not None:
            self.store('simulation', message.pose.pose, message.header,
                       (message.header.frame_id, self.node.board.connection_id))

    def pose(self):
        node = self.node
        error = node.health_error()
        if error:
            raise ValueError(error)
        if self.source == 'simulation':
            if not node.board.simulated:
                raise ValueError('真实控制板禁止使用仿真真值')
        else:
            s = node.navigation
            if s is None or time.monotonic()-node.navigation_time > node.timeout:
                raise ValueError('导航状态过期')
            if not s.usable or not s.origin_ready:
                raise ValueError('导航当前不可用或原点未就绪')
            if not s.extrinsics_calibrated or s.pose_child_frame != 'base_link':
                raise ValueError('自动作业需要标定到 base_link 的位姿')
            if s.simulated and not node.board.simulated:
                raise ValueError('真实控制板禁止使用模拟导航')
            if s.age_ms == 65535 or s.age_ms/1000+time.monotonic()-node.navigation_time > .5:
                raise ValueError('导航测量年龄超过 500 ms')
        sample = self.samples.get(self.source)
        if sample is None or time.monotonic()-sample[1] > node.timeout:
            raise ValueError('未收到新鲜车辆位姿')
        expected = self.nav_identity() if self.source == 'navigation' else (sample[2][0],node.board.connection_id)
        if sample[2] != expected:
            raise ValueError('定位会话已改变，等待本次位姿')
        return sample[0], (self.source, sample[2], node.board.connection_id)

    @property
    def active(self):
        return self.tracker is not None and not self.tracker.done and not self.tracker.fault

    def capture(self):
        if self.node.phase != 'idle' or self.node.enable_future is not None or (self.node.control and self.node.control.enabled):
            raise ValueError('请先停止车辆再建区')
        pose, identity = self.pose()
        self.require_stationary(pose)
        self.anchor, self.identity = pose, identity
        self.plan = self.tracker = None
        self.message = '角点已锁定；拖动对角点，然后生成路线'

    def require_stationary(self, pose):
        history = self.histories.get(self.source, ())
        if len(history)<2 or history[-1][0]-history[0][0]<.4:
            raise ValueError('等待至少 0.4 秒稳定定位后建区/启动')
        if any(p.distance(pose)>.02 or abs(wrap(p.yaw-pose.yaw))>.035 for _,p in history):
            raise ValueError('车辆仍在移动或定位不稳定')

    def generate(self, x, y, width):
        if self.node.phase != 'idle' or self.active:
            raise ValueError('运行中不能修改路线')
        if self.anchor is None:
            raise ValueError('请先以当前位置建区')
        self.plan = plan_rectangle(self.anchor, x, y, width)
        self.message = f'{self.plan.rows} 行 · 行距 {self.plan.spacing:.2f} m · 全程 {self.plan.length:.1f} m'

    def start(self, tools, speed):
        if self.plan is None:
            raise ValueError('请先生成路线')
        pose, identity = self.pose()
        self.require_stationary(pose)
        if identity != self.identity:
            raise ValueError('定位原点或连接改变，请重新建区')
        if pose.distance(self.anchor) > self.position_tolerance or abs(wrap(pose.yaw-self.anchor.yaw)) > self.yaw_tolerance:
            raise ValueError('车辆已离开建区位姿，请重新建区')
        if not self.node.board.capabilities & 1:
            raise ValueError('当前板卡不支持底盘')
        tracker = CoverageTracker(self.plan, speed, self.position_tolerance, self.yaw_tolerance)
        # 仅保留本任务使用的刀盘和升降；升降在各路段保持，刀盘仅覆盖段开启。
        self.tools = Selection(mower_on=tools.mower_on, mower_percent=tools.mower_percent,
                               lift_on=tools.lift_on, lift_mm=tools.lift_mm).normalized()
        self.tools.validate_device(self.node.board.capabilities,self.node.lift_max_mm)
        self.node.start(Selection())
        self.tracker = tracker
        self.message = '正在准备区域作业'

    def cancel(self, reason):
        if self.active:
            self.tracker.abort(reason)
            self.message = '已停止：'+reason

    def tick(self):
        if not self.active:
            return
        try:
            pose, identity = self.pose()
            if identity != self.identity:
                raise ValueError('定位原点、会话或控制板连接改变')
        except ValueError as error:
            self.node.stop(str(error))
            return
        if self.node.phase != 'running':
            return
        output = self.tracker.update(pose,time.monotonic())
        if self.tracker.fault:
            self.message = '已停止：'+self.tracker.fault
            self.node.stop(self.message)
        elif self.tracker.done:
            self.message = '作业完成：已返回初始位置及朝向'
            self.node.stop(self.message)
        else:
            self.node.active = replace(self.tools,linear_mps=output.linear,angular_radps=output.angular,
                                       mower_on=self.tools.mower_on and output.mowing).normalized()
            names = {'entry':'进入首行','coverage':'覆盖作业','turn':'圆弧换行','return':'返回起点','align':'恢复朝向'}
            self.message = f'{names[self.tracker.phase]} · 路段 {self.tracker.index+1}/{len(self.plan.segments)}'
