from __future__ import annotations

import struct
from types import SimpleNamespace

from osi3.osi_groundtruth_pb2 import GroundTruth

from carla_osi_publisher.config import GroundTruthConfig
from carla_osi_publisher.groundtruth import GroundTruthBuilder
from carla_osi_publisher.trace import write_length_prefixed_trace
from carla_osi_publisher.validator import GroundTruthValidator

from .fakes import Attribute, BoundingBox, EnvironmentObject, Extent, FakeActor, FakeWorld, Rotation, Transform, Vector


def test_groundtruth_builder_and_validator() -> None:
    ego = FakeActor(
        7,
        "vehicle.tesla.model3",
        attributes=[
            Attribute("role_name", "hero"),
            Attribute("osi_vehicle_type", "MEDIUM_CAR"),
            Attribute("number_of_wheels", "4"),
        ],
    )
    pedestrian = FakeActor(8, "walker.pedestrian.0001")
    world = FakeWorld([ego, pedestrian])

    result = GroundTruthBuilder(GroundTruthConfig(ego="hero")).build(world)
    message = result.message

    assert message.version.version_major == 3
    assert message.version.version_minor == 8
    assert message.version.version_patch == 0
    assert message.host_vehicle_id.value != 0
    assert len(message.moving_object) == 2
    assert message.moving_object[0].base.position.y == -2.0
    assert message.moving_object[0].base.dimension.length == 4.0

    report = GroundTruthValidator().validate(message, require_host_vehicle=True)
    assert report.valid, report.errors


def test_groundtruth_without_ego_is_reported_only_when_required() -> None:
    world = FakeWorld([FakeActor(1, "vehicle.tesla.model3")])
    message = GroundTruthBuilder(GroundTruthConfig(ego="missing")).build(world).message
    report = GroundTruthValidator().validate(message)
    assert report.valid
    strict_report = GroundTruthValidator().validate(message, require_host_vehicle=True)
    assert not strict_report.valid
    assert "GroundTruth.host_vehicle_id is not set" in strict_report.errors


def test_groundtruth_resolves_mapping_style_carla_attributes() -> None:
    ego = FakeActor(7, "vehicle.tesla.model3")
    ego.attributes = {"role_name": "hero", "osi_vehicle_type": "MEDIUM_CAR"}

    message = GroundTruthBuilder(GroundTruthConfig(ego="hero")).build(
        FakeWorld([ego])
    ).message

    assert message.host_vehicle_id.value != 0
    assert message.moving_object[0].vehicle_classification.type != 0
    assert GroundTruthValidator().validate(message, require_host_vehicle=True).valid


def test_groundtruth_converts_static_sign_and_traffic_light() -> None:
    labels = SimpleNamespace(
        Buildings="Buildings",
        TrafficSigns="TrafficSigns",
    )
    carla_module = SimpleNamespace(CityObjectLabel=labels)
    building = EnvironmentObject(
        20,
        "building_20",
        BoundingBox(Vector(0.0, 0.0, 0.0), Extent(5.0, 4.0, 3.0)),
        Transform(Vector(10.0, 20.0, 3.0), Rotation()),
    )
    sign = EnvironmentObject(
        21,
        "traffic.speed_limit.30",
        BoundingBox(Vector(0.0, 0.0, 0.0), Extent(0.2, 0.2, 1.5)),
        Transform(Vector(11.0, 21.0, 1.5), Rotation(yaw=45.0)),
    )
    traffic_light = FakeActor(22, "traffic.traffic_light")
    traffic_light.get_state = lambda: "Red"
    world = FakeWorld(
        [traffic_light],
        {
            labels.Buildings: [building],
            labels.TrafficSigns: [sign],
        },
    )

    result = GroundTruthBuilder(
        GroundTruthConfig(ego=None),
        carla_module=carla_module,
    ).build(world)
    message = result.message

    assert result.stationary_object_count == 1
    assert result.traffic_sign_count == 1
    assert result.traffic_light_count == 1
    assert message.stationary_object[0].classification.type != 0
    assert message.traffic_sign[0].id.value != 0
    assert message.traffic_sign[0].main_sign.classification.type != 0
    assert message.traffic_sign[0].main_sign.classification.value.value == 30.0
    assert message.traffic_light[0].classification.color != 0
    assert GroundTruthValidator().validate(message).valid


def test_validator_detects_duplicate_ids_across_moving_objects() -> None:
    world = FakeWorld(
        [
            FakeActor(7, "vehicle.tesla.model3"),
            FakeActor(7, "walker.pedestrian.0001"),
        ]
    )
    message = GroundTruthBuilder(GroundTruthConfig(ego=None)).build(world).message

    report = GroundTruthValidator().validate(message)

    assert not report.valid
    assert "moving_object[1].id is duplicated" in report.errors


def test_length_prefixed_trace_can_be_read_back(tmp_path) -> None:
    world = FakeWorld([FakeActor(7, "vehicle.tesla.model3")])
    message = GroundTruthBuilder(GroundTruthConfig(ego=None)).build(world).message
    path = tmp_path / "ground_truth.osi"

    assert write_length_prefixed_trace(path, [message, message]) == 2

    with path.open("rb") as stream:
        payloads = []
        while length_bytes := stream.read(4):
            (length,) = struct.unpack("<I", length_bytes)
            payloads.append(stream.read(length))

    assert len(payloads) == 2
    decoded = GroundTruth()
    decoded.ParseFromString(payloads[0])
    assert decoded.version.version_minor == 8
    assert decoded.timestamp.seconds == 12


def test_large_environment_id_is_compacted_into_osi_id() -> None:
    labels = SimpleNamespace(Buildings="Buildings")
    carla_module = SimpleNamespace(CityObjectLabel=labels)
    building = EnvironmentObject(
        1 << 63,
        "large_id_building",
        BoundingBox(Vector(0.0, 0.0, 0.0), Extent(1.0, 1.0, 1.0)),
        Transform(Vector(1.0, 2.0, 3.0), Rotation()),
    )
    world = FakeWorld([FakeActor(1, "vehicle.tesla.model3")], {labels.Buildings: [building]})

    message = GroundTruthBuilder(
        GroundTruthConfig(ego=None),
        carla_module=carla_module,
    ).build(world).message
    object_id = message.stationary_object[0].id.value

    assert object_id >> 56 == 2
    assert 0 < (object_id & ((1 << 56) - 1)) < (1 << 56)
    assert GroundTruthValidator().validate(message).valid
