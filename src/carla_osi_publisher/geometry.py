"""CARLA to OSI coordinate and geometry helpers."""

from __future__ import annotations

import math
from typing import Any


def _value(obj: Any, name: str, default: float = 0.0) -> float:
    return float(getattr(obj, name, default))


def vector3(message: Any, value: Any, *, flip_y: bool = True) -> None:
    """Copy a CARLA Location/Vector3D into an OSI Vector3d-like message."""

    message.x = _value(value, "x")
    message.y = -_value(value, "y") if flip_y else _value(value, "y")
    message.z = _value(value, "z")


def orientation3d(message: Any, rotation: Any, *, flip_yaw: bool = True) -> None:
    """Copy CARLA degrees into OSI radians."""

    message.roll = math.radians(_value(rotation, "roll"))
    message.pitch = math.radians(_value(rotation, "pitch"))
    yaw = _value(rotation, "yaw")
    message.yaw = math.radians(-yaw if flip_yaw else yaw)


def dimension3d(message: Any, bounding_box: Any) -> None:
    """Copy a CARLA bounding box extent into full OSI dimensions."""

    extent = getattr(bounding_box, "extent", bounding_box)
    message.length = 2.0 * abs(_value(extent, "x"))
    message.width = 2.0 * abs(_value(extent, "y"))
    message.height = 2.0 * abs(_value(extent, "z"))


def mounting_position(message: Any, transform: Any, *, flip_y: bool = True) -> None:
    """Copy a CARLA relative transform into an OSI MountingPosition."""

    vector3(message.position, transform.location, flip_y=flip_y)
    orientation3d(message.orientation, transform.rotation, flip_yaw=flip_y)


def timestamp(message: Any, seconds: float) -> None:
    """Write a floating-point simulation time to an OSI Timestamp."""

    whole = math.floor(float(seconds))
    nanos = round((float(seconds) - whole) * 1_000_000_000)
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    message.seconds = int(whole)
    message.nanos = nanos


def actor_position(actor: Any) -> Any:
    transform = actor.get_transform()
    return transform.location


def actor_rotation(actor: Any) -> Any:
    return actor.get_transform().rotation
