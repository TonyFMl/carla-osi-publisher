from __future__ import annotations

from osi_utilities import MultiTraceReader

from carla_osi_publisher.config import GroundTruthConfig
from carla_osi_publisher.groundtruth import GroundTruthBuilder
from carla_osi_publisher.mcap import DualMcapWriter, McapConversionError, convert_osi_to_mcap
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
