"""独立实车故障注入演示：仅覆盖行内部施加可见的横向参考偏置。"""
import math
from dataclasses import replace
from terramind_panel.coverage import wrap
from terramind_panel.coverage_tracking import CoverageTracker


class DemoTracker(CoverageTracker):
    def __init__(self, *args, amplitude=.12, **kwargs):
        super().__init__(*args, **kwargs)
        if not 0 <= amplitude <= .12:
            raise ValueError('演示偏置幅值必须在 0～0.12 m 内')
        self.amplitude=amplitude
        self.run_started=None
        self.maximum_cross_track=0.

    def update(self, pose, now):
        if self.run_started is None:self.run_started=now
        if now-self.run_started>450:return self.abort('演示总时长上限')
        x,y=self.plan.local(pose.x,pose.y)
        xmin,ymin,xmax,ymax=self.plan.bounds
        if not (xmin-.5 <= x <= xmax+.5 and ymin-.35 <= y <= ymax+.35):
            return self.abort('演示空间边界触发')
        index=self.index
        segment=self.plan.segments[index] if index<len(self.plan.segments) else None
        cross=progress=0.
        if segment is not None and segment.kind=='line':
            dx,dy=pose.x-segment.start.x,pose.y-segment.start.y
            c,s=math.cos(segment.start.yaw),math.sin(segment.start.yaw)
            progress=dx*c+dy*s;cross=-dx*s+dy*c
            self.maximum_cross_track=max(self.maximum_cross_track,abs(cross))
            if abs(cross)>.18:return self.abort('演示横向偏差超过 0.18 m')
        # 原控制器每一拍仍使用真实位姿，执行原有时钟、偏离、到位及停稳检查。
        output=super().update(pose,now)
        if (self.fault or self.done or self.index!=index or not segment or
                not segment.mowing or segment.kind!='line' or output.linear<=0):
            return replace(output,mowing=False)
        # 只在覆盖直线中部注入；起步、终点、掉头、返航沿用正常控制。
        if self.amplitude and .15<progress<segment.length-.45:
            fraction=(progress-.15)/(segment.length-.60)
            bias=self.amplitude*math.sin(2*math.pi*fraction)
            bearing=segment.start.yaw-math.atan2(cross-bias,.25)
            error=wrap(bearing-pose.yaw)
            return replace(output,angular=self._clamp(2.0*error),mowing=False)
        return replace(output,mowing=False)
