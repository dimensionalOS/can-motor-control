"""Type stubs for can_motor_control.damiao."""

from __future__ import annotations

from enum import IntEnum

class MotorType(IntEnum):
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM8006 = 6
    DM8009 = 7
    DM10010L = 8
    DM10010 = 9
    DMH3510 = 10
    DMH6215 = 11
    DMG6220 = 12

class MotorProfile:
    @property
    def id(self) -> int: ...
    @property
    def p_max(self) -> float: ...
    @property
    def v_max(self) -> float: ...
    @property
    def t_max(self) -> float: ...
    def __init__(self, id: int, *, p_max: float, v_max: float, t_max: float) -> None: ...

class DamiaoCodec:
    def __init__(self, *, profiles: list[MotorProfile] = ...) -> None: ...
