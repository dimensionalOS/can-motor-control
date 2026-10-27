"""Damiao vendor codec — exposed as `can_motor_control.damiao.*`."""

from can_motor_control._native import DamiaoCodec, MotorType, MotorProfile

__all__ = ["DamiaoCodec", "MotorType", "MotorProfile"]
