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
    def __init__(self, plan, speed=.15, position_tolerance=.03, yaw_tolerance=.035,
                 minimum_linear=0., minimum_angular=0., maximum_angular=1.2):
        if not all(math.isfinite(v) for v in (speed, position_tolerance, yaw_tolerance)):
            raise ValueError('跟踪参数必须有限')
        if not 0 < speed <= .35 or not 0 < position_tolerance <= .2 or not 0 < yaw_tolerance <= .2:
            raise ValueError('跟踪速度或到位容差超出范围')
        self.plan = plan
        self.speed = speed
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        if not (math.isfinite(minimum_linear) and 0 <= minimum_linear <= speed and
                math.isfinite(minimum_angular) and math.isfinite(maximum_angular) and
                0 <= minimum_angular <= maximum_angular <= 1.2 and maximum_angular > 0):
            raise ValueError('运动死区参数超出范围')
        self.minimum_linear = minimum_linear
        self.minimum_angular = minimum_angular
        self.maximum_angular = maximum_angular
        self.pulse_until = self.rest_until = 0.
        self.pulse_output = TrackingOutput()
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
                return self.abort(f'转向过程中偏离目标位置：{distance:.3f} m，上限 {self.position_tolerance*2:.3f} m')
            # 小动作后留出导航约 200 ms 的测量延迟，避免旧位姿导致反复过调。
            if now < self.rest_until:
                return self.pulse_output if now < self.pulse_until else TrackingOutput()
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
            return self._rotate(angle_error, now)
        if segment.kind == 'line':
            dx, dy = pose.x-segment.start.x, pose.y-segment.start.y
            c, s = math.cos(segment.start.yaw), math.sin(segment.start.yaw)
            progress = dx*c+dy*s
            cross = -dx*s+dy*c
            if abs(cross) > .5 or progress < -.5 or progress > segment.length+.5:
                return self.abort('偏离规划直线')
            if now < self.rest_until:
                return self.pulse_output if now < self.pulse_until else TrackingOutput()
            # 路段必须走到终点附近，不能通过跳到后续点略过覆盖线。
            end_tolerance = min(self.position_tolerance, segment.length*.1)
            if self.minimum_linear:
                end_tolerance = min(self.position_tolerance, segment.length*.25)
            if self.index+1 < len(self.plan.segments):
                following = self.plan.segments[self.index+1]
                if following.kind == 'arc':
                    # 小半径掉头不能在离弧线起点一个半径的距离就结束直线。
                    end_tolerance = min(end_tolerance, following.radius*.15)
            if distance <= end_tolerance:
                self._advance(now)
                return TrackingOutput()
            remaining = segment.length-progress
            lookahead = .35 if self.minimum_linear else .18
            gain = 1.5 if self.minimum_linear else 3.
            bearing = (math.atan2(segment.end.y-pose.y, segment.end.x-pose.x)
                       if remaining < .15 else segment.start.yaw-math.atan2(cross,lookahead))
            error = wrap(bearing-pose.yaw)
            v = min(self.speed, .8*distance)*max(0., math.cos(error))
            if abs(error) > .7:
                return self._rotate(error, now)
            if self.minimum_linear:
                v = max(self.minimum_linear, v)
                # 端点附近用短距离动作，停稳后才根据新测量决定是否继续。
                if distance < .15:
                    output = TrackingOutput(v, self._clamp(gain*error), segment.mowing)
                    return self._pulse(output, max(.12,min(.3,distance/v*.4)), now)
            return TrackingOutput(v, self._clamp(gain*error), segment.mowing)
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
        return TrackingOutput(v, self._clamp(omega), False)

    def _clamp(self, omega):
        return max(-self.maximum_angular, min(self.maximum_angular, omega))

    def _pulse(self, output, duration, now):
        self.pulse_output = output
        self.pulse_until = now+duration
        self.rest_until = self.pulse_until+.45
        return output

    def _rotate(self, error, now):
        omega = self._clamp(2.5*error)
        if self.minimum_angular:
            omega = math.copysign(max(self.minimum_angular,abs(omega)),error)
            if abs(error)<.3:
                return self._pulse(TrackingOutput(0.,omega), max(.12,min(.25,abs(error)/abs(omega)*.5)), now)
        return TrackingOutput(0.,omega)

    def _advance(self, now):
        self.index += 1
        self.started = now
        self.settled_since = None
        if self.minimum_linear or self.minimum_angular:
            self.pulse_until = now
            self.rest_until = now+.45
        if self.index == len(self.plan.segments):
            self.done = True
