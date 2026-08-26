from __future__ import annotations

from types import SimpleNamespace

from carla_osi_publisher.config import GroundTruthConfig
from carla_osi_publisher.streaming import StreamingUpdateBuilder

from .fakes import BoundingBox, EnvironmentObject, Extent, FakeActor, FakeWorld, Rotation, Transform, Vector


def test_streaming_initial_frame_contains_static_objects_only_once() -> None:
    labels = SimpleNamespace(Buildings="Buildings", TrafficSigns="TrafficSigns")
    carla_module = SimpleNamespace(CityObjectLabel=labels)
    building = EnvironmentObject(
        100,
        "building",
        BoundingBox(Vector(0.0, 0.0, 0.0), Extent(2.0, 2.0, 2.0)),
        Transform(Vector(1.0, 2.0, 2.0), Rotation()),
    )
    sign = EnvironmentObject(
        101,
        "traffic.stop",
        BoundingBox(Vector(0.0, 0.0, 0.0), Extent(0.2, 0.2, 1.0)),
        Transform(Vector(2.0, 3.0, 1.0), Rotation()),
    )
    actor = FakeActor(7, "vehicle.tesla.model3")
    world = FakeWorld(
        [actor],
        {
            labels.Buildings: [building],
            labels.TrafficSigns: [sign],
        },
    )
    builder = StreamingUpdateBuilder(
        GroundTruthConfig(ego=None),
        carla_module=carla_module,
    )

    first = builder.build(world)
    second = builder.build(world)

    assert first.initial
    assert not second.initial
    assert len(first.message.stationary_object_update) == 1
    assert len(first.message.traffic_sign_update) == 1
    assert len(first.message.moving_object_update) == 1
    assert len(second.message.stationary_object_update) == 0
    assert len(second.message.traffic_sign_update) == 0
    assert len(second.message.moving_object_update) == 1


def test_streaming_update_reports_removed_dynamic_object() -> None:
    actor = FakeActor(7, "vehicle.tesla.model3")
    world = FakeWorld([actor])
    builder = StreamingUpdateBuilder(GroundTruthConfig(ego=None))

    builder.build(world)
    world._actors = []
    second = builder.build(world)

    assert len(second.message.moving_object_update) == 0
    assert [item.value for item in second.message.obsolete_id] == [(1 << 56) | 7]
