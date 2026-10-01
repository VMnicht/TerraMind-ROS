from dataclasses import replace
import math

import pytest

from terramind_panel.controls import Selection


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
