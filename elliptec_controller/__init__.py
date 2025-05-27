"""
Elliptec Controller Package

A Python package for controlling Thorlabs Elliptec rotation stages.
Provides both individual rotator control and synchronized group movement capabilities.
"""

from .controller import (
    ElliptecRotator,
    ElliptecGroupController,
    degrees_to_hex,
    hex_to_degrees,
    COMMAND_GET_STATUS,
    COMMAND_STOP,
    COMMAND_HOME,
    COMMAND_FORWARD,
    COMMAND_BACKWARD,
    COMMAND_MOVE_ABS,
    COMMAND_MOVE_REL,
    COMMAND_GET_POS,
    COMMAND_SET_VELOCITY,
    COMMAND_GET_VELOCITY,
    COMMAND_SET_HOME_OFFSET,
    COMMAND_GET_HOME_OFFSET,
    COMMAND_SET_JOG_STEP,
    COMMAND_GET_JOG_STEP,
    COMMAND_GET_INFO,
    COMMAND_OPTIMIZE_MOTORS,
    COMMAND_GROUP_ADDRESS
)

__all__ = [
    "ElliptecRotator",
    "ElliptecGroupController",
    "degrees_to_hex",
    "hex_to_degrees",
    "COMMAND_GET_STATUS",
    "COMMAND_STOP",
    "COMMAND_HOME",
    "COMMAND_FORWARD",
    "COMMAND_BACKWARD",
    "COMMAND_MOVE_ABS",
    "COMMAND_MOVE_REL",
    "COMMAND_GET_POS",
    "COMMAND_SET_VELOCITY",
    "COMMAND_GET_VELOCITY",
    "COMMAND_SET_HOME_OFFSET",
    "COMMAND_GET_HOME_OFFSET",
    "COMMAND_SET_JOG_STEP",
    "COMMAND_GET_JOG_STEP",
    "COMMAND_GET_INFO",
    "COMMAND_OPTIMIZE_MOTORS",
    "COMMAND_GROUP_ADDRESS",
]

__version__ = "0.2.0"
