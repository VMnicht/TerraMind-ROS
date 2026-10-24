"""覆盖轨迹的低速差速跟踪；外部必须先验证定位新鲜度和控制权限。"""
import math
from dataclasses import dataclass
from .coverage import wrap


@dataclass(frozen=True)
class TrackingOutput:
    linear: float = 0.
    angular: float = 0.
    mowing: bool = False


class CoverageTracker:
    def __init__(self, plan, speed=.15, position_tolerance=.03, yaw_tolerance=.035):
        if not all(math.isfinite(v) for v in (speed, position_tolerance, yaw_tolerance)):
            raise ValueError('跟踪参数必须有限')
        if not 0 < speed <= .35 or not 0 < position_tolerance <= .2 or not 0 < yaw_tolerance <= .2:
            raise ValueError('跟踪速度或到位容差超出范围')
        self.plan = plan
        self.speed = speed
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.index = 0
        self.done = False
        self.fault = ''
        self.settled_since = None
        self.started = self.last_time = None

    @property
    def phase(self):
        if self.fault:
            return 'fault'
        if self.done:
            return 'complete'
        return self.plan.segments[self.index].phase

    def abort(self, reason):
        self.fault = reason
        return TrackingOutput()

    def update(self, pose, now):
        if self.done or self.fault:
            return TrackingOutput()
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, now)):
            return self.abort('位姿或时钟无效')
        if self.last_time is not None and (now < self.last_time or now-self.last_time > .15):
            return self.abort('跟踪调度超时')
        self.last_time = now
        if self.started is None:
            self.started = now
        segment = self.plan.segments[self.index]
        if now-self.started > max(30., segment.length/self.speed*12):
            return self.abort('路段到位超时')
        distance = pose.distance(segment.end)
        angle_error = wrap(segment.end.yaw-pose.yaw)
        if segment.kind == 'rotate':
            if distance > self.position_tolerance*2:
                return self.abort('转向过程中偏离目标位置')
            if abs(angle_error) <= self.yaw_tolerance:
                if segment.phase == 'align' and distance > self.position_tolerance:
                    return self.abort('最终位置不满足到位容差')
                if self.settled_since is None:
                    self.settled_since = now
                if now-self.settled_since >= (.8 if segment.phase == 'align' else .15):
                    if segment.phase == 'align' and distance > self.position_tolerance:
                        return self.abort('最终位置不满足到位容差')
                    self._advance(now)
                return TrackingOutput()
            self.settled_since = None
            return TrackingOutput(0., max(-1.2, min(1.2, 2.5*angle_error)))
        if segment.kind == 'line':
            dx, dy = pose.x-segment.start.x, pose.y-segment.start.y
            c, s = math.cos(segment.start.yaw), math.sin(segment.start.yaw)
            progress = dx*c+dy*s
            cross = -dx*s+dy*c
            # 路段必须走到终点附近，不能通过跳到后续点略过覆盖线。
            if distance <= min(self.position_tolerance, segment.length*.1):
                self._advance(now)
                return TrackingOutput()
            if abs(cross) > .5 or progress < -.5 or progress > segment.length+.5:
                return self.abort('偏离规划直线')
            remaining = segment.length-progress
            bearing = (math.atan2(segment.end.y-pose.y, segment.end.x-pose.x)
                       if remaining < .15 else segment.start.yaw-math.atan2(cross,.18))
            error = wrap(bearing-pose.yaw)
            v = min(self.speed, .8*distance)*max(0., math.cos(error))
            if abs(error) > .7:
                v = 0.
            return TrackingOutput(v, max(-1.2,min(1.2,3.*error)), segment.mowing)
        # 圆弧沿径向投影跟踪，前馈角速度 v/r，方向由 sweep 给出。
        radial = math.atan2(pose.y-segment.center[1],pose.x-segment.center[0])
        initial = math.atan2(segment.start.y-segment.center[1],segment.start.x-segment.center[0])
        sign = math.copysign(1.,segment.sweep)
        swept = sign*wrap(radial-initial)
        # 直线段在容差内结束时，投影可能略早于弧线起点，不能误当成绕行一圈。
        if swept < -math.pi/2:
            swept += 2*math.pi
        if distance <= min(self.position_tolerance,segment.radius*.15) and swept > math.pi*.8:
            self._advance(now)
            return TrackingOutput()
        radius = math.hypot(pose.x-segment.center[0],pose.y-segment.center[1])
        if abs(radius-segment.radius) > .5 or swept > math.pi+.3:
            return self.abort('偏离规划掉头弧线')
        error = wrap(radial+sign*math.pi/2-pose.yaw)
        v = min(self.speed, .6*segment.radius)
        omega = v*segment.curvature+3.*error+sign*2.*(radius-segment.radius)
        return TrackingOutput(v, max(-1.2,min(1.2,omega)), False)

    def _advance(self, now):
        self.index += 1
        self.started = now
        self.settled_since = None
        if self.index == len(self.plan.segments):
            self.done = True
