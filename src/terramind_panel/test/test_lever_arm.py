import math
import numpy as np
import pytest
from terramind_panel.lever_arm import base_yaw, estimate, SpinCalibration


def circle(offset, direction=1, center=(2,-3), noise=0., yaw0=2.7):
    yaw = np.linspace(yaw0,yaw0+direction*2*math.pi,721)
    c,s = np.cos(yaw),np.sin(yaw)
    xy = np.column_stack([center[0]+c*offset[0]-s*offset[1],center[1]+s*offset[0]+c*offset[1]])
    xy += np.random.default_rng(7).normal(0,noise,xy.shape)
    return np.column_stack([xy,np.arctan2(s,c)]).tolist()


@pytest.mark.parametrize('offset',[(.8,.13),(.45,-.2),(-.3,.4),(0.,0.)])
@pytest.mark.parametrize('noise',[0.,.005])
def test_fit_both_directions_and_different_centers(offset,noise):
    result = estimate(circle(offset,noise=noise),circle(offset,-1,center=(2.02,-2.99),noise=noise))
    assert np.linalg.norm(np.array([result.x,result.y])-offset)<.002
    assert result.rms<.01
    assert result.direction_difference<.002


@pytest.mark.parametrize('failure',['partial','few','same_direction','slip','disagree','nan','gaps'])
def test_bad_trajectories_rejected(failure):
    a,b = circle((.7,.1)),circle((.7,.1),-1)
    if failure=='partial':a=a[:500]
    if failure=='few':a=a[::30]
    if failure=='same_direction':b=circle((.7,.1))
    if failure=='slip':
        for i,row in enumerate(a):row[0]+=.3*i/len(a)
    if failure=='disagree':b=circle((.8,.2),-1)
    if failure=='nan':a[100][0]=float('nan')
    if failure=='gaps':a=a[:100]+a[300:]
    with pytest.raises(ValueError):estimate(a,b)


def test_mounting_yaw_and_invalid_quaternions():
    yaw,mount = 2.9,.35
    assert abs(base_yaw([math.cos((yaw+mount)/2),0,0,math.sin((yaw+mount)/2)],
                        [math.cos(mount/2),0,0,math.sin(mount/2)])-yaw)<1e-12
    for q in ([0,0,0,0],[float('nan'),0,0,0],[math.cos(.1),math.sin(.1),0,0]):
        with pytest.raises(ValueError):base_yaw(q,[1,0,0,0])


def test_spin_closed_loop_with_acceleration_and_delayed_measurements():
    # 含加速度限制和 200 ms 传输延迟，初始航向跨过 ±pi。
    yaw=2.9;omega=0.;dt=.02;now=0.;pending=[]
    spin=SpinCalibration(yaw,now)
    command=0.
    for _ in range(4500):
        now+=dt
        omega+=max(-4*dt,min(4*dt,command-omega));yaw+=omega*dt
        c,s=math.cos(yaw),math.sin(yaw)
        pending.append((.8*c-.12*s,.8*s+.12*c,math.atan2(s,c)))
        if len(pending)>10:
            command=spin.step(*pending.pop(0),now)
        if spin.done:break
    assert spin.done
    assert abs(yaw-2.9)<.02
    result=estimate(*spin.samples)
    assert abs(result.x-.8)<1e-8 and abs(result.y-.12)<1e-8


def test_spin_timeout_and_jump():
    spin=SpinCalibration(0.,0.)
    with pytest.raises(ValueError,match='超时'):spin.step(0,0,0,100)
    with pytest.raises(ValueError,match='跳变'):SpinCalibration(0.,0.).step(0,0,1,.02)


# 使用真实 ROS 消息类型检查硬件接入门控；无需物理串口。
from test_coverage_runtime import FakeNode, message
from terramind_panel.lever_arm_runtime import LeverArmRuntime
from terramind_panel.controls import Selection


@pytest.fixture
def runtime(monkeypatch):
    monkeypatch.setattr('terramind_panel.lever_arm_runtime.time.monotonic',lambda:100.)
    node=FakeNode();node.input_conflict=lambda:''
    node.navigation.nav_valid=True;node.navigation.nav_fault=False
    node.navigation.gnss_recent=True;node.navigation.rtk_state=4
    node.navigation.extrinsics_calibrated=False;node.navigation.pose_child_frame='navigation_imu'
    runtime=LeverArmRuntime(node,[1,0,0,0])
    for t in np.linspace(99.4,100.,31):runtime.receive(message(t))
    return runtime


def test_uncalibrated_navigation_can_start(runtime):
    runtime.start()
    assert runtime.running and runtime.node.phase=='running'
    assert runtime.node.active==Selection()


def test_200hz_navigation_retains_stationary_window(runtime):
    high_rate=LeverArmRuntime(runtime.node,[1,0,0,0])
    for t in np.linspace(99.4,100.,121):high_rate.receive(message(t))
    high_rate.start()
    assert high_rate.running and runtime.node.phase=='running'


@pytest.mark.parametrize('fault',['session','origin','connection','stale','invalid','age','rtk','simulated','conflict','tilt'])
def test_runtime_fault_stops_and_never_resumes(runtime,fault):
    runtime.start();node=runtime.node
    if fault=='session':node.navigation.session_id+=1
    if fault=='origin':node.navigation.origin_height_m+=1
    if fault=='connection':node.board.connection_id+=1
    if fault=='stale':node.navigation_time=99.
    if fault=='invalid':node.navigation.nav_valid=False
    if fault=='age':node.navigation.age_ms=65535
    if fault=='rtk':node.navigation.rtk_state=5
    if fault=='simulated':node.navigation.simulated=True
    if fault=='conflict':node.input_conflict=lambda:'other publisher'
    if fault=='tilt':
        m=message(100.01);m.pose.orientation.w=math.cos(.1);m.pose.orientation.x=math.sin(.1)
        runtime.receive(m)
    runtime.tick()
    assert runtime.error and not runtime.running and runtime.result is None
    assert node.phase=='idle' and node.active==Selection()
    node.navigation_time=100.;node.navigation.nav_valid=True
    runtime.receive(message(100.02));runtime.tick()
    assert not runtime.running and node.phase=='idle'


def test_duplicates_do_not_refresh_and_motion_blocks_start(runtime):
    old=runtime.sample
    runtime.receive(message(100.,5.));assert runtime.sample==old
    runtime.receive(message(100.01,.1))
    with pytest.raises(ValueError,match='停稳'):runtime.start()
