# 完整目标校验：覆盖边界值、能力限制及关闭装置归零，同时保留原草稿。
from dataclasses import replace
import math

import pytest

from terramind_panel.controls import Selection, validate_lift_limit


def test_all_fields_and_off_normalization():
    draft = Selection(.35, -2., True, -500., True, 500., True, 100., True, 1000., True, 100.)
    draft.validate(63)
    with pytest.raises(ValueError, match='升降'):
        draft.validate(15)
    closed = replace(draft, lift_on=False, sprayer_on=False)
    closed.validate(15)
    wire = closed.normalized()
    assert wire.lift_mm == wire.sprayer_percent == 0
    assert closed.lift_mm == 1000  # Editing a draft must not mutate the active snapshot.
    assert wire.left_rpm == -500 and wire.right_rpm == 500


@pytest.mark.parametrize('key,value', [
    ('linear_mps', .351), ('linear_mps', -.351), ('angular_radps', 2.01),
    ('left_rpm', 501), ('right_rpm', -501), ('mower_percent', -1),
    ('lift_mm', 1001), ('sprayer_percent', 101), ('linear_mps', math.nan),
    ('left_rpm', math.inf),
])
def test_rejects_values_outside_protocol(key, value):
    with pytest.raises(ValueError):
        replace(Selection(), **{key: value}).validate(63)


def test_missing_chassis_capability():
    Selection().validate(0)
    with pytest.raises(ValueError, match='底盘'):
        Selection(linear_mps=.1).validate(62)


def test_sprayer_capability_and_off_normalization():
    draft = Selection(sprayer_on=True, sprayer_percent=37.5)
    draft.validate(0x002f)
    with pytest.raises(ValueError, match='喷洒'):
        draft.validate(0x000f)
    with pytest.raises(ValueError, match='升降'):
        replace(draft, lift_on=True).validate(0x002f)
    closed = replace(draft, sprayer_on=False)
    assert closed.normalized().sprayer_percent == 0
    assert closed.sprayer_percent == 37.5


def test_device_limit_does_not_change_wire_protocol():
    Selection(lift_on=True, lift_mm=38.56).validate_device(63)
    Selection(lift_on=True, lift_mm=1000).validate(63)
    with pytest.raises(ValueError, match='机构行程'):
        Selection(lift_on=True, lift_mm=38.57).validate_device(63)
    with pytest.raises(ValueError, match='机构行程'):
        Selection(lift_on=True, lift_mm=21).validate_device(63, 20)
    for limit in (0, -1, 1000, math.nan, math.inf):
        with pytest.raises(ValueError):
            validate_lift_limit(limit)


def test_navigation_status_never_claims_stale_data_usable():
    from types import SimpleNamespace
    from terramind_panel.navigation_status import describe_navigation
    assert '等待' in describe_navigation(None, False)
    state = SimpleNamespace(
        online=True, usable=True, origin_ready=True, extrinsics_calibrated=False,
        age_ms=200, rtk_state=4, time_locked=True, gnss_recent=True,
        pose_child_frame='navigation_imu', velocity_point_frame='navigation_imu',
        header=SimpleNamespace(frame_id='navigation_map'), stamp_reference='receive_time',
        session_id=1, packet_seq=5, simulated=True, received_frames=5, crc_errors=0,
        missing_packets=0, board_restarts=0, reconnects=0, reason='ok',
        origin_latitude_rad=.5, origin_longitude_rad=2., origin_height_m=20.)
    assert '导航可用：是' in describe_navigation(state, True)
    stale = describe_navigation(state, False)
    assert '导航可用：否' in stale and '已过期' in stale
    state.origin_ready = False
    state.age_ms = 65535
    text = describe_navigation(state, True)
    assert '导航可用：否' in text and '年龄：未知' in text
