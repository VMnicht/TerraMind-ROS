import pytest
from start_delay import DelayedStart


def fixture():
    events=[]
    healthy=[True]
    def validate():
        if not healthy[0]:raise ValueError('定位失效')
    delay=DelayedStart(lambda:events.append('start'),validate,events.append)
    return delay,events,healthy


def test_waits_full_delay_and_starts_only_once():
    delay,events,_=fixture();delay.arm(10,0)
    for i in range(100):delay.tick(i/10)
    assert delay.pending and events==[]
    delay.tick(10);delay.tick(10.1)
    assert events==['start'] and not delay.pending


def test_cancel_never_restarts():
    delay,events,_=fixture();delay.arm(10,0);delay.cancel();delay.tick(20)
    assert events==[] and not delay.pending


def test_lost_health_cancels_without_automatic_restart():
    delay,events,healthy=fixture();delay.arm(10,0)
    healthy[0]=False;delay.tick(.1);healthy[0]=True;delay.tick(10)
    assert len(events)==1 and '定位失效' in events[0] and not delay.pending


def test_stalled_ui_does_not_start_after_deadline():
    delay,events,_=fixture();delay.arm(10,0);delay.tick(11)
    assert len(events)==1 and '响应超时' in events[0] and not delay.pending


def test_zero_delay_still_validates():
    delay,events,healthy=fixture();delay.arm(0,0);healthy[0]=False;delay.tick(.01)
    assert 'start' not in events and not delay.pending


@pytest.mark.parametrize('seconds',[-1,121,float('nan'),float('inf')])
def test_invalid_delay(seconds):
    delay,events,_=fixture()
    with pytest.raises(ValueError):delay.arm(seconds,0)
    assert not delay.pending and events==[]


def test_start_failure_cancels():
    delay,events,_=fixture()
    def fail():raise ValueError('启动位姿不符')
    delay.start=fail;delay.arm(0,0);delay.tick(.01)
    assert not delay.pending and '启动位姿不符' in events[0]
