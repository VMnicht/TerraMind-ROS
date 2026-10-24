"""定位与控制门控测试；真实消息类型，假时钟与假控制后端。"""
from types import SimpleNamespace as NS
import pytest
from geometry_msgs.msg import PoseStamped
from terramind_panel.coverage_runtime import CoverageRuntime
from terramind_panel.controller import PanelNode
from terramind_panel.controls import Selection


class FakeNode:
    def __init__(self):
        self.timeout=.15;self.phase='idle';self.enable_future=None;self.lift_max_mm=38.56
        self.board=NS(simulated=False,connection_id=7,capabilities=63)
        self.control=NS(enabled=False)
        self.navigation=NS(header=NS(frame_id='field'),session_id=11,origin_latitude_rad=.5,
            origin_longitude_rad=2.,origin_height_m=20.,pose_child_frame='base_link',usable=True,
            origin_ready=True,extrinsics_calibrated=True,simulated=False,age_ms=200)
        self.navigation_time=100.;self.active=Selection();self.reason=''
        self.coverage=CoverageRuntime(self)

    def declare_parameter(self,name,value):return NS(value=value)
    def create_subscription(self,*args):pass
    def health_error(self):return ''
    def _source_time(self,header):return header.stamp.sec+header.stamp.nanosec*1e-9
    def start(self,draft):self.phase='running';self.control.enabled=True
    def stop(self,reason):
        self.coverage.cancel(reason);self.reason=reason;self.phase='idle';self.active=Selection()


def message(t=100.,x=0.,frame='field'):
    m=PoseStamped();m.header.frame_id=frame;m.header.stamp.sec=int(t)
    m.header.stamp.nanosec=int((t-int(t))*1e9)
    m.pose.orientation.w=1.;m.pose.position.x=x
    return m


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr('terramind_panel.coverage_runtime.time.monotonic',lambda:100.)
    n=FakeNode()
    for t in (99.5,99.7,99.9,100.):n.coverage.on_navigation(message(t))
    return n


def start(node):
    r=node.coverage;r.capture();r.generate(2.,1.,.4);r.start(Selection(mower_on=True,mower_percent=8.),.15)
    assert r.active
    return r


@pytest.mark.parametrize('field,value', [('usable',False),('origin_ready',False),
    ('extrinsics_calibrated',False),('pose_child_frame','navigation_imu'),('simulated',True),('age_ms',65535),('age_ms',501)])
def test_navigation_quality_blocks_execution(node,field,value):
    setattr(node.navigation,field,value)
    with pytest.raises(ValueError):node.coverage.capture()


@pytest.mark.parametrize('change', ['pose_stale','status_stale','session','origin','connection','source'])
def test_loss_or_identity_change_stops_without_resume(node,change):
    r=start(node)
    if change=='pose_stale':r.samples['navigation']=(r.anchor,99.,r.nav_identity())
    if change=='status_stale':node.navigation_time=99.
    if change=='session':node.navigation.session_id+=1
    if change=='origin':node.navigation.origin_height_m+=1
    if change=='connection':node.board.connection_id+=1
    if change=='source':r.source='simulation'
    r.tick()
    assert node.phase=='idle' and r.tracker.fault and not r.tracker.done
    node.navigation_time=100.
    r.source='navigation';r.on_navigation(message(100.01))
    r.tick()
    assert node.phase=='idle' and not r.active and node.active==Selection()


def test_pose_validation_duplicate_and_wrong_frame(node):
    r=node.coverage;old=r.samples['navigation']
    r.on_navigation(message(100.,5.))
    r.on_navigation(message(100.01,5.,'other'))
    bad=message(100.02);bad.pose.orientation.w=0.;r.on_navigation(bad)
    assert r.samples['navigation']==old


def test_anchor_requires_stationary_and_start_checks_original_pose(node):
    r=node.coverage;r.histories['navigation'].clear()
    with pytest.raises(ValueError,match='稳定'):r.capture()
    for t in (99.5,99.7,100.):r.histories['navigation'].append((t,r.samples['navigation'][0]))
    r.capture();r.generate(2.,1.,.4)
    r.on_navigation(message(100.01,.3))
    with pytest.raises(ValueError):r.start(Selection(),.15)
    assert node.phase=='idle'


def test_real_board_rejects_simulation_source(node):
    node.coverage.source='simulation'
    with pytest.raises(ValueError,match='真实控制板'):node.coverage.pose()


def test_manual_commands_cannot_override_coverage(node):
    start(node)
    with pytest.raises(ValueError,match='手动'):PanelNode.start(node,Selection())
    with pytest.raises(ValueError,match='手动'):PanelNode.apply(node,Selection())
    with pytest.raises(ValueError):node.coverage.generate(3.,2.,.5)
    node.stop('user stop')
    assert not node.coverage.active and not node.coverage.tracker.done
