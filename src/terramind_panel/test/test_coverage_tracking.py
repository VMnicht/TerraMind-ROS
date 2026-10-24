import math
import pytest
from terramind_panel.coverage import Pose2, plan_rectangle, wrap
from terramind_panel.coverage_tracking import CoverageTracker


@pytest.mark.parametrize('corner,width', [((1.2,1.),.4), ((-1.2,1.),.4),
                                           ((1.2,-1.),.4), ((-1.2,-1.),.4),
                                           ((.8,.2),.5), ((1.,.8),.4),
                                           ((.2,.1),.05), ((.02,.02),.4), ((2.,1.),.33)])
def test_closed_loop_return(corner,width):
    initial = Pose2(12.,-8.,.63)
    plan = plan_rectangle(initial,*corner,width)
    tracker = CoverageTracker(plan)
    pose = initial
    phases = set()
    for tick in range(60000):
        phases.add(tracker.phase)
        command = tracker.update(pose,tick*.02)
        assert abs(command.linear) <= .35 and abs(command.angular) <= 1.2
        assert not tracker.fault, tracker.fault
        pose = Pose2(pose.x+command.linear*math.cos(pose.yaw)*.02,
                     pose.y+command.linear*math.sin(pose.yaw)*.02,
                     wrap(pose.yaw+command.angular*.02))
        if tracker.done:
            break
    assert tracker.done
    assert pose.distance(initial) <= tracker.position_tolerance
    assert abs(wrap(pose.yaw-initial.yaw)) <= tracker.yaw_tolerance
    assert {'entry','coverage','return','align'} <= phases


def test_stall_latches_failure():
    origin=Pose2(0,0,0)
    tracker=CoverageTracker(plan_rectangle(origin,2,1,.4))
    tracker.update(origin,0.)
    assert tracker.update(origin,.3).linear == 0
    assert tracker.fault and not tracker.done
    assert tracker.update(origin,.31).angular == 0
