import math
import pytest
from terramind_panel.coverage import Pose2,plan_rectangle
from tracker import DemoTracker


def tracker():
    return DemoTracker(plan_rectangle(Pose2(0,0,0),2,2,.27,.2,'pivot'),.1,.06,.07,.1,.6,.6)


def test_real_deviation_stops_before_demo_injection():
    t=tracker();out=t.update(Pose2(.5,.19,0),0.)
    assert t.fault and out.linear==out.angular==0


def test_faults_and_clock_stall_are_preserved():
    t=tracker();t.update(Pose2(.5,0,0),0.)
    assert t.update(Pose2(.5,0,0),.3).linear==0 and t.fault


def test_output_limits_and_implements_off():
    t=tracker()
    for i in range(100):
        out=t.update(Pose2(.2+i*.01,0,0),i*.02)
        assert 0<=out.linear<=.1 and abs(out.angular)<=.6 and not out.mowing
    assert not t.fault


def test_invalid_amplitude_rejected():
    for amplitude in (-.1,.13,math.nan,math.inf):
        with pytest.raises(ValueError):
            DemoTracker(tracker().plan,amplitude=amplitude)


def test_zero_injection_keeps_normal_tracking_output():
    from terramind_panel.coverage_tracking import CoverageTracker
    plan=tracker().plan
    normal=CoverageTracker(plan,.1,.06,.07,.1,.6,.6)
    demo=DemoTracker(plan,.1,.06,.07,.1,.6,.6,amplitude=0.)
    for i in range(100):
        pose=Pose2(.2+i*.01,.01,0.)
        a=normal.update(pose,i*.02);b=demo.update(pose,i*.02)
        assert (a.linear,a.angular)==(b.linear,b.angular)
        assert not b.mowing
