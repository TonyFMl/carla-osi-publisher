"""Lazy access to OSI 3.8.0 protobuf bindings."""

from __future__ import annotations

from importlib import import_module
from typing import Any

from .errors import CarlaUnavailableError
from .version import OSI_VERSION


def osi_message(module_name: str, class_name: str) -> type[Any]:
    """Import an OSI protobuf class with a focused error message."""

    try:
        module = import_module(f"osi3.{module_name}")
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "OSI Python bindings are unavailable. Install the workspace "
            "dependency with `uv sync`, or install osi-python 3.8.0."
        ) from exc
    return getattr(module, class_name)


def set_interface_version(message: Any) -> None:
    """Set the OSI interface version on a top-level message."""

    if not hasattr(message, "version"):
        return
    message.version.version_major = OSI_VERSION.major
    message.version.version_minor = OSI_VERSION.minor
    message.version.version_patch = OSI_VERSION.patch


def ensure_carla() -> Any:
    """Import CARLA only when a live simulator operation is requested."""

    try:
        return import_module("carla")
    except ModuleNotFoundError as exc:
        raise CarlaUnavailableError(
            "The CARLA Python API is not installed. Install the CARLA 0.9.16 "
            "wheel/egg supplied with the simulator."
        ) from exc
