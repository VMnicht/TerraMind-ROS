import math
import pytest
from terramind_panel.coverage import Pose2, plan_rectangle, wrap
from terramind_panel.coverage_tracking import CoverageTracker


@pytest.mark.parametrize('corner,width', [((1.2,1.),.4), ((-1.2,1.),.4),
                                           ((1.2,-1.),.4), ((-1.2,-1.),.4),
                                           ((.8,.2),.5), ((1.,.8),.4),
                                           ((.2,.1),.05), ((.02,.02),.4), ((2.,1.),.33), ((3.,3.),.27)])
@pytest.mark.parametrize('offset', [0., .2])
def test_closed_loop_return(corner,width,offset):
    initial = Pose2(12.,-8.,.63)
    plan = plan_rectangle(initial,*corner,width,offset)
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
    assert {'coverage','return','align'} <= phases


def test_stall_latches_failure():
    origin=Pose2(0,0,0)
    tracker=CoverageTracker(plan_rectangle(origin,2,1,.4))
    tracker.update(origin,0.)
    assert tracker.update(origin,.3).linear == 0
    assert tracker.fault and not tracker.done
    assert tracker.update(origin,.31).angular == 0


@pytest.mark.parametrize('speed,left_gain,right_gain,noise', [(.2,.90,.92,0.),(.1,1.72,1.78,.008)])
def test_field_profile_completes_with_motor_deadband_and_navigation_delay(speed,left_gain,right_gain,noise):
    from collections import deque
    pose=Pose2(0.,0.,0.)
    plan=plan_rectangle(pose,2.,2.,.27,.2,'pivot')
    tracker=CoverageTracker(plan,speed,.06,.07,.1,.6,.6)
    history=deque([pose]*11,maxlen=11)
    left=right=0.
    for tick in range(30000):
        q=history[0]
        measured=Pose2(q.x+noise*math.sin(tick*.07),q.y+noise*math.cos(tick*.04),
                       q.yaw+noise*.375*math.sin(tick*.05))
        command=tracker.update(measured,tick*.02)
        assert 0 <= command.linear <= speed and abs(command.angular) <= .6
        assert not tracker.fault,tracker.fault
        targets=(command.linear-.16*command.angular,command.linear+.16*command.angular)
        # 旧固件 32 cm 轮距、实车 38 cm；电机死区、左右响应差及惯性。
        l=targets[0]*left_gain if abs(targets[0])>=.06 else 0.
        r=targets[1]*right_gain if abs(targets[1])>=.06 else 0.
        left+=(l-left)*.12;right+=(r-right)*.12
        v=(left+right)/2;omega=(right-left)/.38
        pose=Pose2(pose.x+v*math.cos(pose.yaw)*.02,pose.y+v*math.sin(pose.yaw)*.02,
                   wrap(pose.yaw+omega*.02))
        history.append(pose)
        if tracker.done:break
    assert tracker.done
    assert pose.distance(plan.origin)<=.06
    assert abs(wrap(pose.yaw))<=.07


def test_pulse_still_checks_pose_and_clock_faults():
    origin=Pose2(0.,0.,0.)
    plan=plan_rectangle(origin,2.,2.,.27,.2,'pivot')
    tracker=CoverageTracker(plan,.2,.06,.07,.1,.6,.6)
    tracker.index=next(i for i,s in enumerate(plan.segments) if s.kind=='rotate')
    target=plan.segments[tracker.index].end
    near=Pose2(target.x,target.y,target.yaw-.2)
    assert tracker.update(near,0.).angular == .6
    assert tracker.update(Pose2(1.,1.,near.yaw),.02).angular == 0.
    assert tracker.fault
