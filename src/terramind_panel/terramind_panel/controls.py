"""协议范围与不可变完整目标快照；不依赖 ROS，可单独测试。"""
from dataclasses import dataclass, replace
import math

LEGACY_CAPABILITIES = 0x000f
CURRENT_CAPABILITIES = 0x002f
ALL_CAPABILITIES = 0x003f
# 当前机构安全显示上限，向下取整到面板的 0.01 mm 精度；协议仍为 0..1000。
DEFAULT_LIFT_MAX_MM = 38.56


def validate_lift_limit(value):
    if not math.isfinite(value) or not 0 < value <= DEFAULT_LIFT_MAX_MM:
        raise ValueError('lift_max_mm must be in (0, 38.56] for the current mechanism')
    return value


@dataclass(frozen=True)
class ActuatorField:
    key: str
    title: str
    value_field: str
    unit: str
    minimum: float
    maximum: float
    step: float
    bit: int


ACTUATORS = (
    # bit 为能力位的位号（不是掩码）；bit0 由底盘使用。
    ActuatorField('left', '左播撒', 'left_rpm', 'RPM', -500, 500, 10, 1),
    ActuatorField('right', '右播撒', 'right_rpm', 'RPM', -500, 500, 10, 2),
    ActuatorField('mower', '割草刀盘', 'mower_percent', '%', 0, 100, 1, 3),
    ActuatorField('lift', '升降缩短量', 'lift_mm', 'mm', 0, 1000, 1, 4),
    ActuatorField('sprayer', '喷洒装置', 'sprayer_percent', '%', 0, 100, 1, 5),
)


@dataclass(frozen=True)
class Selection:
    # 同一类型用于草稿和生效目标，冻结后不会被界面编辑原地改动。
    linear_mps: float = 0.0
    angular_radps: float = 0.0
    left_on: bool = False
    left_rpm: float = 0.0
    right_on: bool = False
    right_rpm: float = 0.0
    mower_on: bool = False
    mower_percent: float = 0.0
    lift_on: bool = False
    lift_mm: float = 0.0
    sprayer_on: bool = False
    sprayer_percent: float = 0.0

    def validate(self, capabilities):
        for label, value, low, high in (
                ('线速度', self.linear_mps, -.35, .35),
                ('角速度', self.angular_radps, -2., 2.)):
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{label}超出范围 [{low}, {high}]')
        if not capabilities & 1 and (self.linear_mps or self.angular_radps):
            raise ValueError('当前板卡不支持底盘运动')
        for field in ACTUATORS:
            value = getattr(self, field.value_field)
            if not math.isfinite(value) or not field.minimum <= value <= field.maximum:
                raise ValueError(f'{field.title}目标超出协议范围')
            if getattr(self, field.key + '_on') and not capabilities & (1 << field.bit):
                raise ValueError(f'当前板卡不支持{field.title}')

    def normalized(self):
        # 保留草稿中的编辑值，只在新生成的发送快照里把关闭装置的目标归零。
        changes = {f.value_field: 0.0 for f in ACTUATORS if not getattr(self, f.key + '_on')}
        return replace(self, **changes)

    def validate_device(self, capabilities, lift_max_mm=DEFAULT_LIFT_MAX_MM):
        self.validate(capabilities)
        validate_lift_limit(lift_max_mm)
        if self.lift_on and self.lift_mm > lift_max_mm:
            raise ValueError('升降目标超出当前机构行程')
