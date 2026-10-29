"""Kaixuan KGU actuator support for Better Actuator Models (BAM)."""

from .actuator import KguActuator
from .loader import load_kgu_model, register_kgu_actuator

register_kgu_actuator()

__all__ = ["KguActuator", "load_kgu_model", "register_kgu_actuator"]
