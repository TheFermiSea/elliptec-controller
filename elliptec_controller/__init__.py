"""
Elliptec Controller Package

This package provides classes and utilities for controlling Thorlabs Elliptec rotators.
"""

from .controller import (
    ElliptecRotator,
    TripleRotatorController,
    degrees_to_hex,
    hex_to_degrees,
    COMMAND_GET_STATUS,
    COMMAND_STOP,
    COMMAND_HOME,
    COMMAND_FORWARD,
    COMMAND_BACKWARD,
    COMMAND_MOVE_ABS,
    COMMAND_GET_POS,
    COMMAND_SET_VELOCITY,
    COMMAND_GET_VELOCITY,
    COMMAND_SET_HOME_OFFSET,
    COMMAND_GET_HOME_OFFSET,
    COMMAND_SET_JOG_STEP,
    COMMAND_GET_JOG_STEP,
    COMMAND_GET_INFO,
    COMMAND_OPTIMIZE_MOTORS
)

__all__ = [
    "ElliptecRotator",
    "TripleRotatorController",
    "degrees_to_hex",
    "hex_to_degrees",
    "COMMAND_GET_STATUS",
    "COMMAND_STOP",
    "COMMAND_HOME",
    "COMMAND_FORWARD",
    "COMMAND_BACKWARD",
    "COMMAND_MOVE_ABS",
    "COMMAND_GET_POS",
    "COMMAND_SET_VELOCITY",
    "COMMAND_GET_VELOCITY",
    "COMMAND_SET_HOME_OFFSET",
    "COMMAND_GET_HOME_OFFSET",
    "COMMAND_SET_JOG_STEP",
    "COMMAND_GET_JOG_STEP",
    "COMMAND_GET_INFO",
    "COMMAND_OPTIMIZE_MOTORS",
]

__version__ = '0.1.0'
