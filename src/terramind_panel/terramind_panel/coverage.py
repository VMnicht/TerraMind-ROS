"""矩形覆盖几何核心；坐标单位米、角度弧度，不依赖 ROS 或 Qt。

矩形是覆盖范围，允许掉头越界。起点是车辆参考点，不是车辆包络角点。
转向段显式保留，不能用一串位置点丢掉初末朝向要求。
"""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float

    def distance(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class Segment:
    kind: str                       # line / arc / rotate
    phase: str                      # entry / coverage / turn / return / align
    start: Pose2
    end: Pose2
    mowing: bool = False
    center: tuple = (0., 0.)
    sweep: float = 0.

    @property
    def radius(self):
        return math.hypot(self.start.x - self.center[0], self.start.y - self.center[1])

    @property
    def length(self):
        if self.kind == 'rotate':
            return 0.
        return self.radius * abs(self.sweep) if self.kind == 'arc' else self.start.distance(self.end)

    @property
    def curvature(self):
        return math.copysign(1. / self.radius, self.sweep) if self.kind == 'arc' else 0.

    def at(self, fraction):
        t = max(0., min(1., fraction))
        if t == 0.:
            return self.start
        if t == 1.:
            return self.end
        if self.kind == 'arc':
            angle = math.atan2(self.start.y-self.center[1], self.start.x-self.center[0]) + self.sweep*t
            return Pose2(self.center[0]+self.radius*math.cos(angle),
                         self.center[1]+self.radius*math.sin(angle), wrap(self.start.yaw+self.sweep*t))
        return Pose2(self.start.x+(self.end.x-self.start.x)*t,
                     self.start.y+(self.end.y-self.start.y)*t,
                     wrap(self.start.yaw+wrap(self.end.yaw-self.start.yaw)*t))


@dataclass(frozen=True)
class CoveragePlan:
    origin: Pose2
    corner_x: float                  # 对角点在初始车体坐标系的有符号坐标
    corner_y: float
    width: float
    rows: int
    spacing: float
    segments: tuple

    @property
    def length(self):
        return sum(s.length for s in self.segments)

    def world(self, x, y):
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        return (self.origin.x+c*x-s*y, self.origin.y+s*x+c*y)

    def local(self, x, y):
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        dx, dy = x-self.origin.x, y-self.origin.y
        return c*dx+s*dy, -s*dx+c*dy

    def preview(self, step=.08):
        if not math.isfinite(step) or step <= 0:
            raise ValueError('预览采样间距必须为正数')
        # 大区域仍使用解析几何；这里只限制绘图点密度。
        step = max(step, self.length / 12000.)
        return tuple(tuple(segment.at(i/n) for i in range(n+1))
                     for segment in self.segments
                     for n in [max(1, math.ceil(segment.length/step))])


def plan_rectangle(origin, corner_x, corner_y, width):
    """四个拖拽象限均支持；最后一个位姿精确等于输入初始位姿。"""
    if not all(math.isfinite(v) for v in (origin.x, origin.y, origin.yaw, corner_x, corner_y, width)):
        raise ValueError('位姿、矩形尺寸和覆盖宽度必须为有限数')
    if width <= 0 or abs(corner_x) < .001 or abs(corner_y) < .001:
        raise ValueError('覆盖宽度必须大于零，矩形两边至少为 1 mm')
    rows = math.ceil(abs(corner_y)/width)
    if rows > 2000:
        raise ValueError('覆盖行数超过 2000，请检查尺寸与覆盖宽度')
    spacing = abs(corner_y)/rows
    sx, sy = math.copysign(1., corner_x), math.copysign(1., corner_y)
    result = []
    current = origin
    c, s = math.cos(origin.yaw), math.sin(origin.yaw)

    def world(x, y):
        return origin.x+c*x-s*y, origin.y+s*x+c*y

    def rotate(yaw, phase):
        nonlocal current
        target = Pose2(current.x, current.y, yaw)
        if abs(wrap(yaw-current.yaw)) > 1e-10 or phase == 'align':
            result.append(Segment('rotate', phase, current, target))
        current = target

    def line(x, y, phase, mowing=False):
        nonlocal current
        if math.hypot(x-current.x, y-current.y) < 1e-10:
            return
        rotate(math.atan2(y-current.y, x-current.x), phase)
        target = Pose2(x, y, current.yaw)
        result.append(Segment('line', phase, current, target, mowing))
        current = target

    line(*world(0., sy*spacing/2), 'entry')
    for i in range(rows):
        y = sy*(i+.5)*spacing
        end_x = corner_x if i % 2 == 0 else 0.
        line(*world(end_x, y), 'coverage', True)
        if i + 1 < rows:
            center = world(end_x, y+sy*spacing/2)
            sweep = math.pi*sx*sy*(1 if i % 2 == 0 else -1)
            target = Pose2(*world(end_x, y+sy*spacing), wrap(current.yaw+sweep))
            result.append(Segment('arc', 'turn', current, target, False, center, sweep))
            current = target
    line(origin.x, origin.y, 'return')
    rotate(origin.yaw, 'align')
    return CoveragePlan(origin, corner_x, corner_y, width, rows, spacing, tuple(result))
