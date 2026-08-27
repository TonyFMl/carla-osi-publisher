from __future__ import annotations

from osi_utilities import MultiTraceReader

from carla_osi_publisher.config import GroundTruthConfig
from carla_osi_publisher.groundtruth import GroundTruthBuilder
from carla_osi_publisher.mcap import (
    DualMcapWriter,
    McapConversionError,
    SensorViewMcapWriter,
    SupportedMessagesMcapWriter,
    convert_osi_to_mcap,
)
from carla_osi_publisher.sensorview import (
    CameraSensorConfig,
    CameraSensorViewBuilder,
    LidarSensorConfig,
    LidarSensorViewBuilder,
)
from carla_osi_publisher.streaming import StreamingUpdateBuilder
from carla_osi_publisher.trace import write_length_prefixed_trace

from .fakes import FakeActor, FakeWorld


def test_convert_groundtruth_osi_to_mcap(tmp_path) -> None:
    input_path = tmp_path / "ground_truth.osi"
    output_path = tmp_path / "ground_truth.mcap"
    world = FakeWorld([FakeActor(7, "vehicle.tesla.model3")])
    message = GroundTruthBuilder(GroundTruthConfig(ego=None)).build(world).message
    write_length_prefixed_trace(input_path, [message, message])

    result = convert_osi_to_mcap(
        input_path,
        output_path,
        input_type="GroundTruth",
        topic="ground_truth",
        compression="none",
    )

    assert result.message_count == 2
    assert result.message_type == "GroundTruth"
    assert output_path.is_file()

    reader = MultiTraceReader()
    assert reader.open(output_path)
    assert reader.get_available_topics() == ["ground_truth"]
    file_metadata = reader.get_file_metadata()[0]["data"]
    channel_metadata = reader.get_channel_metadata("ground_truth")
    messages = list(reader)
    reader.close()

    assert file_metadata["min_osi_version"] == "3.8.0"
    assert file_metadata["max_osi_version"] == "3.8.0"
    assert channel_metadata is not None
    assert channel_metadata["net.asam.osi.trace.channel.osi_version"] == "3.8.0"
    assert len(messages) == 2
    assert messages[0].message is not None
    assert len(messages[0].message.moving_object) == 1


def test_convert_rejects_unknown_message_type(tmp_path) -> None:
    input_path = tmp_path / "ground_truth.osi"
    output_path = tmp_path / "ground_truth.mcap"
    input_path.write_bytes(b"")

    try:
        convert_osi_to_mcap(input_path, output_path, input_type="NotAnOSIMessage")
    except McapConversionError as exc:
        assert "Unsupported OSI message type" in str(exc)
    else:
        raise AssertionError("Expected McapConversionError")


def test_convert_rejects_non_mcap_output(tmp_path) -> None:
    input_path = tmp_path / "ground_truth.osi"
    input_path.write_bytes(b"")

    try:
        convert_osi_to_mcap(input_path, tmp_path / "output.bin")
    except McapConversionError as exc:
        assert ".mcap extension" in str(exc)
    else:
        raise AssertionError("Expected McapConversionError")


def test_dual_mcap_writer_contains_groundtruth_and_streaming_channels(tmp_path) -> None:
    output_path = tmp_path / "dual.mcap"
    world = FakeWorld([FakeActor(7, "vehicle.tesla.model3")])
    ground_truth = GroundTruthBuilder(GroundTruthConfig(ego=None)).build(world).message
    streaming_update = StreamingUpdateBuilder(GroundTruthConfig(ego=None)).build(world).message

    with DualMcapWriter(output_path, compression="none") as writer:
        writer.write(ground_truth, streaming_update)

    reader = MultiTraceReader()
    assert reader.open(output_path)
    assert reader.get_available_topics() == ["ground_truth", "streaming_update"]
    assert reader.get_channel_metadata("ground_truth")["net.asam.osi.trace.channel.osi_version"] == "3.8.0"
    assert (
        reader.get_channel_metadata("streaming_update")["net.asam.osi.trace.channel.osi_version"]
        == "3.8.0"
    )
    messages = list(reader)
    reader.close()

    assert len(messages) == 2
    assert messages[0].message.DESCRIPTOR.full_name == "osi3.GroundTruth"
    assert messages[1].message.DESCRIPTOR.full_name == "osi3.StreamingUpdate"


def test_sensorview_mcap_writer_contains_camera_topics(tmp_path) -> None:
    output_path = tmp_path / "sensor_view.mcap"
    host_vehicle = FakeActor(7, "vehicle.tesla.model3")
    image = type(
        "Image",
        (),
        {
            "width": 2,
            "height": 1,
            "timestamp": 1.25,
            "raw_data": bytes([1, 2, 3, 4, 10, 20, 30, 40]),
        },
    )()
    first = CameraSensorViewBuilder(
        CameraSensorConfig(name="camera_left", width=2, height=1, yaw=-90.0)
    ).build(image, type("Sensor", (), {"id": 11})(), host_vehicle)
    second = CameraSensorViewBuilder(
        CameraSensorConfig(name="camera_front", width=2, height=1)
    ).build(image, type("Sensor", (), {"id": 12})(), host_vehicle)

    with SensorViewMcapWriter(output_path, compression="none") as writer:
        writer.write(first, "sensor_view/camera_left")
        writer.write(second, "sensor_view/camera_front")

    reader = MultiTraceReader()
    assert reader.open(output_path)
    assert reader.get_available_topics() == [
        "sensor_view/camera_left",
        "sensor_view/camera_front",
    ]
    messages = list(reader)
    reader.close()

    assert len(messages) == 2
    assert all(
        result.message.DESCRIPTOR.full_name == "osi3.SensorView"
        for result in messages
    )


def test_sensorview_mcap_writer_contains_lidar_topic(tmp_path) -> None:
    import struct

    output_path = tmp_path / "sensor_view_lidar.mcap"
    host_vehicle = FakeActor(7, "vehicle.tesla.model3")
    measurement = type(
        "LidarMeasurement",
        (),
        {
            "timestamp": 1.25,
            "frame": 10,
            "raw_data": struct.pack("<8f", 4.0, 0.0, 0.0, 0.5, 0.0, 2.0, 1.0, 0.25),
        },
    )()
    message = LidarSensorViewBuilder(
        LidarSensorConfig(name="lidar_0", channels=2)
    ).build(measurement, type("Sensor", (), {"id": 11})(), host_vehicle)

    with SensorViewMcapWriter(output_path, compression="none") as writer:
        writer.write(message, "sensor_view/lidar_0")

    reader = MultiTraceReader()
    assert reader.open(output_path)
    assert reader.get_available_topics() == ["sensor_view/lidar_0"]
    result = next(iter(reader))
    reader.close()

    assert result.message.DESCRIPTOR.full_name == "osi3.SensorView"
    assert len(result.message.lidar_sensor_view[0].reflection) == 2


def test_supported_messages_writer_contains_all_supported_topics(tmp_path) -> None:
    import struct

    output_path = tmp_path / "all_supported.mcap"
    host_vehicle = FakeActor(7, "vehicle.tesla.model3")
    world = FakeWorld([host_vehicle])
    ground_truth = GroundTruthBuilder(GroundTruthConfig()).build(world).message
    streaming_update = StreamingUpdateBuilder(GroundTruthConfig()).build(world).message
    image = type(
        "Image",
        (),
        {
            "width": 1,
            "height": 1,
            "timestamp": 1.25,
            "raw_data": bytes([1, 2, 3, 4]),
        },
    )()
    camera = CameraSensorViewBuilder(
        CameraSensorConfig(name="camera_front", width=1, height=1)
    ).build(image, type("Sensor", (), {"id": 11})(), host_vehicle)
    lidar = LidarSensorViewBuilder(
        LidarSensorConfig(name="lidar_0", channels=2)
    ).build(
        type(
            "LidarMeasurement",
            (),
            {
                "timestamp": 1.25,
                "raw_data": struct.pack("<4f", 4.0, 0.0, 0.0, 0.5),
            },
        )(),
        type("Sensor", (), {"id": 12})(),
        host_vehicle,
    )

    with SupportedMessagesMcapWriter(output_path, compression="none") as writer:
        writer.write(ground_truth, "ground_truth")
        writer.write(streaming_update, "streaming_update")
        writer.write(camera, "sensor_view/camera_front")
        writer.write(lidar, "sensor_view/lidar_0")

    reader = MultiTraceReader()
    assert reader.open(output_path)
    assert reader.get_available_topics() == [
        "ground_truth",
        "streaming_update",
        "sensor_view/camera_front",
        "sensor_view/lidar_0",
    ]
    messages = list(reader)
    reader.close()

    assert len(messages) == 4
    message_types = {
        result.channel_name: result.message.DESCRIPTOR.full_name for result in messages
    }
    assert message_types == {
        "ground_truth": "osi3.GroundTruth",
        "streaming_update": "osi3.StreamingUpdate",
        "sensor_view/camera_front": "osi3.SensorView",
        "sensor_view/lidar_0": "osi3.SensorView",
    }
