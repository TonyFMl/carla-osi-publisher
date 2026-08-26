from __future__ import annotations

from types import SimpleNamespace

from carla_osi_publisher.geometry import dimension3d, orientation3d, timestamp, vector3


def test_geometry_conversion() -> None:
    vector = SimpleNamespace()
    vector3(vector, SimpleNamespace(x=1, y=2, z=3))
    assert (vector.x, vector.y, vector.z) == (1.0, -2.0, 3.0)

    orientation = SimpleNamespace()
    orientation3d(orientation, SimpleNamespace(roll=0, pitch=90, yaw=180))
    assert round(orientation.pitch, 6) == 1.570796
    assert round(orientation.yaw, 6) == -3.141593

    dimension = SimpleNamespace()
    dimension3d(dimension, SimpleNamespace(extent=SimpleNamespace(x=2, y=1, z=0.5)))
    assert (dimension.length, dimension.width, dimension.height) == (4.0, 2.0, 1.0)

    stamp = SimpleNamespace()
    timestamp(stamp, 12.25)
    assert (stamp.seconds, stamp.nanos) == (12, 250_000_000)
