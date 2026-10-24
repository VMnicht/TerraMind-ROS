"""解析几何验收：覆盖、连续性、掉头朝向、初末位姿和输入边界。"""
import math
import pytest
from terramind_panel.coverage import Pose2, plan_rectangle, wrap


@pytest.mark.parametrize('corner', [(4., 2.), (-4., 2.), (4., -2.), (-4., -2.)])
@pytest.mark.parametrize('yaw', [0., .71, -2.2])
@pytest.mark.parametrize('width', [.6, 1., 3.])
def test_rectangle_geometry(corner, yaw, width):
    start = Pose2(21., -17., yaw)
    plan = plan_rectangle(start, *corner, width)
    assert plan.segments[0].start == start
    assert plan.segments[-1].end == start
    assert plan.rows == math.ceil(abs(corner[1])/width)
    assert plan.spacing <= width
    lanes = [s for s in plan.segments if s.mowing]
    assert len(lanes) == plan.rows
    # 检查矩形横向所有采样点均落入至少一个作业条带。
    centers = [plan.local(s.start.x, s.start.y)[1] for s in lanes]
    for i in range(101):
        y = corner[1]*i/100
        assert min(abs(y-cy) for cy in centers) <= width/2+1e-9
    for lane in lanes:
        assert lane.phase == 'coverage' and lane.length == pytest.approx(abs(corner[0]))
        assert sorted([plan.local(p.x, p.y)[0] for p in (lane.start, lane.end)]) == pytest.approx(sorted([0., corner[0]]))
    for a, b in zip(plan.segments, plan.segments[1:]):
        assert a.end.distance(b.start) < 1e-9
        assert abs(wrap(a.end.yaw-b.start.yaw)) < 1e-9
    arcs = [s for s in plan.segments if s.kind == 'arc']
    assert len(arcs) == plan.rows-1
    for arc in arcs:
        assert arc.radius == pytest.approx(plan.spacing/2)
        assert not arc.mowing
        for t in (.001, .3, .8, .999):
            p, q = arc.at(t), arc.at(t+1e-6)
            tangent = math.atan2(q.y-p.y, q.x-p.x)
            assert abs(wrap(tangent-p.yaw)) < 1e-4
    assert all(not s.mowing for s in plan.segments if s.phase != 'coverage')


@pytest.mark.parametrize('x,y,w', [(0,1,1), (1,0,1), (1,1,0), (1,1,-1),
                                    (math.nan,1,1), (1,math.inf,1), (1,1,math.nan),
                                    (1,100,0.0001)])
def test_reject_invalid_or_unbounded_plans(x,y,w):
    with pytest.raises(ValueError):
        plan_rectangle(Pose2(0,0,0),x,y,w)


def test_preview_and_rigid_transform():
    plan = plan_rectangle(Pose2(100,-200,.83),3,2,.4)
    assert plan.local(*plan.world(1.2,-.3)) == pytest.approx((1.2,-.3))
    preview = plan.preview()
    assert preview[0][0] == preview[-1][-1] == plan.origin
    assert all(points[0] == segment.start and points[-1] == segment.end
               for points,segment in zip(preview,plan.segments))
    with pytest.raises(ValueError):
        plan.preview(0)
