"""User-defined firmware scaling through the public Python API."""

import numpy as np
import pytest

import can_motor_control as can
from can_motor_control.damiao import DamiaoCodec, MotorProfile, MotorType


def build(profiles, motor_type):
    return (
        can.Robot.builder()
        .add_bus("bus", can.MockCanBus("custom"), DamiaoCodec(profiles=profiles))
        .add_arm("arm", bus="bus", motors=[can.MotorSpec("joint", motor_type, 1, 0x11)])
        .build()
    )


def test_custom_profile_command_scaling_and_builtin_compatibility():
    profile = MotorProfile(0x8000, p_max=6.0, v_max=10.0, t_max=120.0)
    assert (profile.id, profile.p_max, profile.v_max, profile.t_max) == (
        0x8000,
        6.0,
        10.0,
        120.0,
    )
    with build([profile], profile) as robot:
        arm = robot["arm"]
        arm.mit_control(np.array([[0.0, 0.0, 6.0, 10.0, 120.0]]))
        with pytest.raises(can.CodecError):
            arm.mit_control(np.array([[0.0, 0.0, 6.1, 0.0, 0.0]]))
    with build([], MotorType.DM4310) as robot:
        robot["arm"].mit_control(np.array([[0.0, 0.0, 12.5, 30.0, 10.0]]))


def test_profiles_are_codec_local_and_must_be_registered():
    profile = MotorProfile(0x8000, p_max=6.0, v_max=10.0, t_max=120.0)
    with pytest.raises(can.DmError, match="does not support motor type"):
        build([], profile)
    with pytest.raises(ValueError, match="duplicate"):
        DamiaoCodec(profiles=[profile, profile])
    with pytest.raises(AttributeError):
        profile.p_max = 12.5


@pytest.mark.parametrize("field", ["p_max", "v_max", "t_max"])
@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf"), 1.7e308])
def test_invalid_ranges(field, value):
    ranges = dict(p_max=6.0, v_max=10.0, t_max=120.0)
    ranges[field] = value
    with pytest.raises(ValueError):
        MotorProfile(0x8000, **ranges)


@pytest.mark.parametrize("id", [0, 3, 0x7FFF])
def test_custom_ids_cannot_override_builtin_space(id):
    with pytest.raises(ValueError):
        MotorProfile(id, p_max=6.0, v_max=10.0, t_max=120.0)
