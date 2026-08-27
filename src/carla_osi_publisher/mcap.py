"""MCAP conversion using asam-osi-utilities."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .version import OSI_VERSION


class McapConversionError(RuntimeError):
    """Raised when an OSI to MCAP conversion cannot be completed."""


@dataclass(frozen=True, slots=True)
class McapConversionResult:
    """Summary of one OSI to MCAP conversion."""

    input_path: Path
    output_path: Path
    message_type: str
    topic: str
    message_count: int


class DualMcapWriter:
    """Write complete GroundTruth and incremental StreamingUpdate channels."""

    def __init__(
        self,
        path: str | Path,
        *,
        compression: str | None = None,
        chunk_size: int | None = None,
        description: str | None = None,
        ground_truth_topic: str = "ground_truth",
        streaming_topic: str = "streaming_update",
    ) -> None:
        self.path = Path(path)
        self.compression = compression
        self.chunk_size = chunk_size
        self.description = description
        self.ground_truth_topic = ground_truth_topic
        self.streaming_topic = streaming_topic
        self._writer: Any | None = None
        self._channel_metadata: dict[str, str] | None = None
        self.ground_truth_count = 0
        self.streaming_update_count = 0

    def open(self) -> None:
        if self.path.suffix.lower() != ".mcap":
            raise McapConversionError(f"Dual channel output must use the .mcap extension: {self.path}")

        try:
            from google.protobuf import __version__ as protobuf_version
            from osi_utilities import MultiTraceWriter
            from osi_utilities.tracefile import prepare_required_file_metadata
        except ModuleNotFoundError as exc:
            raise McapConversionError(
                "asam-osi-utilities is unavailable. Run `uv sync` in the project."
            ) from exc

        metadata = prepare_required_file_metadata()
        metadata.update(
            {
                "min_osi_version": OSI_VERSION.as_string(),
                "max_osi_version": OSI_VERSION.as_string(),
                "description": self.description or "CARLA GroundTruth and StreamingUpdate capture",
                "data_sources": "CARLA OSI Publisher",
                "zero_time": "0",
            }
        )
        self._channel_metadata = {
            "net.asam.osi.trace.channel.osi_version": OSI_VERSION.as_string(),
            "net.asam.osi.trace.channel.protobuf_version": protobuf_version,
        }

        writer = MultiTraceWriter()
        writer_kwargs: dict[str, Any] = {}
        if self.compression is not None:
            writer_kwargs["compression"] = self.compression
        if self.chunk_size is not None:
            writer_kwargs["chunk_size"] = self.chunk_size
        if not writer.open(self.path, metadata=metadata, **writer_kwargs):
            raise McapConversionError(f"Could not open output MCAP file: {self.path}")
        self._writer = writer

    def write(self, ground_truth: Any, streaming_update: Any) -> None:
        if self._writer is None or self._channel_metadata is None:
            raise McapConversionError("Dual MCAP writer is not open")

        if self.ground_truth_count == 0:
            ground_truth_metadata = dict(self._channel_metadata)
            ground_truth_metadata["net.asam.osi.trace.channel.description"] = "GroundTruth messages"
            self._writer.add_channel(
                self.ground_truth_topic,
                type(ground_truth),
                metadata=ground_truth_metadata,
            )
        if self.streaming_update_count == 0:
            streaming_metadata = dict(self._channel_metadata)
            streaming_metadata["net.asam.osi.trace.channel.description"] = "StreamingUpdate messages"
            self._writer.add_channel(
                self.streaming_topic,
                type(streaming_update),
                metadata=streaming_metadata,
            )

        if not self._writer.write_message(ground_truth, self.ground_truth_topic):
            raise McapConversionError(
                f"Could not write GroundTruth message {self.ground_truth_count} to {self.path}"
            )
        if not self._writer.write_message(streaming_update, self.streaming_topic):
            raise McapConversionError(
                f"Could not write StreamingUpdate message {self.streaming_update_count} to {self.path}"
            )
        self.ground_truth_count += 1
        self.streaming_update_count += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
            self._writer = None

    def __enter__(self) -> DualMcapWriter:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()


class SensorViewMcapWriter:
    """Write camera and LiDAR SensorView topics to an OSI MCAP file."""

    def __init__(
        self,
        path: str | Path,
        *,
        compression: str | None = None,
        chunk_size: int | None = None,
        description: str | None = None,
    ) -> None:
        self.path = Path(path)
        self.compression = compression
        self.chunk_size = chunk_size
        self.description = description
        self._writer: Any | None = None
        self._channel_metadata: dict[str, str] | None = None
        self._topics: set[str] = set()
        self.message_count = 0

    def open(self) -> None:
        if self.path.suffix.lower() != ".mcap":
            raise McapConversionError(f"SensorView output must use the .mcap extension: {self.path}")

        try:
            from google.protobuf import __version__ as protobuf_version
            from osi_utilities import MultiTraceWriter
            from osi_utilities.tracefile import prepare_required_file_metadata
        except ModuleNotFoundError as exc:
            raise McapConversionError(
                "asam-osi-utilities is unavailable. Run `uv sync` in the project."
            ) from exc

        metadata = prepare_required_file_metadata()
        metadata.update(
            {
                "min_osi_version": OSI_VERSION.as_string(),
                "max_osi_version": OSI_VERSION.as_string(),
                "description": self.description or "CARLA camera and LiDAR SensorView capture",
                "data_sources": "CARLA OSI Publisher",
                "zero_time": "0",
            }
        )
        self._channel_metadata = {
            "net.asam.osi.trace.channel.osi_version": OSI_VERSION.as_string(),
            "net.asam.osi.trace.channel.protobuf_version": protobuf_version,
        }

        writer = MultiTraceWriter()
        writer_kwargs: dict[str, Any] = {}
        if self.compression is not None:
            writer_kwargs["compression"] = self.compression
        if self.chunk_size is not None:
            writer_kwargs["chunk_size"] = self.chunk_size
        if not writer.open(self.path, metadata=metadata, **writer_kwargs):
            raise McapConversionError(f"Could not open output MCAP file: {self.path}")
        self._writer = writer

    def write(self, message: Any, topic: str) -> None:
        if self._writer is None or self._channel_metadata is None:
            raise McapConversionError("SensorView MCAP writer is not open")
        if topic not in self._topics:
            channel_metadata = dict(self._channel_metadata)
            channel_metadata["net.asam.osi.trace.channel.description"] = (
                f"SensorView messages for {topic}"
            )
            self._writer.add_channel(
                topic,
                type(message),
                metadata=channel_metadata,
            )
            self._topics.add(topic)
        if not self._writer.write_message(message, topic):
            raise McapConversionError(
                f"Could not write SensorView message {self.message_count} to {self.path}"
            )
        self.message_count += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
            self._writer = None

    def __enter__(self) -> SensorViewMcapWriter:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()


class SupportedMessagesMcapWriter:
    """Write every currently supported OSI message type to one MCAP file."""

    def __init__(
        self,
        path: str | Path,
        *,
        compression: str | None = None,
        chunk_size: int | None = None,
        description: str | None = None,
    ) -> None:
        self.path = Path(path)
        self.compression = compression
        self.chunk_size = chunk_size
        self.description = description
        self._writer: Any | None = None
        self._channel_metadata: dict[str, str] | None = None
        self._topics: set[str] = set()
        self.message_counts: dict[str, int] = {}
        self.message_count = 0

    def open(self) -> None:
        if self.path.suffix.lower() != ".mcap":
            raise McapConversionError(
                f"All-supported output must use the .mcap extension: {self.path}"
            )

        try:
            from google.protobuf import __version__ as protobuf_version
            from osi_utilities import MultiTraceWriter
            from osi_utilities.tracefile import prepare_required_file_metadata
        except ModuleNotFoundError as exc:
            raise McapConversionError(
                "asam-osi-utilities is unavailable. Run `uv sync` in the project."
            ) from exc

        metadata = prepare_required_file_metadata()
        metadata.update(
            {
                "min_osi_version": OSI_VERSION.as_string(),
                "max_osi_version": OSI_VERSION.as_string(),
                "description": self.description
                or "CARLA all supported OSI message capture",
                "data_sources": "CARLA OSI Publisher",
                "zero_time": "0",
            }
        )
        self._channel_metadata = {
            "net.asam.osi.trace.channel.osi_version": OSI_VERSION.as_string(),
            "net.asam.osi.trace.channel.protobuf_version": protobuf_version,
        }

        writer = MultiTraceWriter()
        writer_kwargs: dict[str, Any] = {}
        if self.compression is not None:
            writer_kwargs["compression"] = self.compression
        if self.chunk_size is not None:
            writer_kwargs["chunk_size"] = self.chunk_size
        if not writer.open(self.path, metadata=metadata, **writer_kwargs):
            raise McapConversionError(f"Could not open output MCAP file: {self.path}")
        self._writer = writer

    def write(self, message: Any, topic: str, *, description: str | None = None) -> None:
        if self._writer is None or self._channel_metadata is None:
            raise McapConversionError("Supported-message MCAP writer is not open")
        if not topic:
            raise McapConversionError("MCAP topic must not be empty")

        if topic not in self._topics:
            channel_metadata = dict(self._channel_metadata)
            channel_metadata["net.asam.osi.trace.channel.description"] = (
                description or f"OSI messages for {topic}"
            )
            self._writer.add_channel(
                topic,
                type(message),
                metadata=channel_metadata,
            )
            self._topics.add(topic)

        if not self._writer.write_message(message, topic):
            raise McapConversionError(
                f"Could not write message {self.message_count} to {self.path}"
            )
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
        self.message_count += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
            self._writer = None

    def __enter__(self) -> SupportedMessagesMcapWriter:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()


def convert_osi_to_mcap(
    input_path: str | Path,
    output_path: str | Path,
    *,
    input_type: str = "GroundTruth",
    topic: str = "ground_truth",
    compression: str | None = None,
    chunk_size: int | None = None,
    description: str | None = None,
) -> McapConversionResult:
    """Convert a single-channel OSI trace into an OSI-compliant MCAP file.

    ``asam-osi-utilities`` supplies the binary trace reader and MCAP writer.
    The converter fills the OSI file and channel metadata with the fixed
    protocol version used by this project.
    """

    source = Path(input_path)
    target = Path(output_path)
    if not source.is_file():
        raise McapConversionError(f"Input OSI trace does not exist: {source}")
    if target.suffix.lower() != ".mcap":
        raise McapConversionError(f"Output file must use the .mcap extension: {target}")
    if not topic:
        raise McapConversionError("MCAP topic must not be empty")

    try:
        from google.protobuf import __version__ as protobuf_version
        from osi_utilities import MessageType, MultiTraceWriter, SingleTraceReader
        from osi_utilities.message_types import require_message_type
        from osi_utilities.tracefile import prepare_required_file_metadata
    except ModuleNotFoundError as exc:
        raise McapConversionError(
            "asam-osi-utilities is unavailable. Run `uv sync` in the project."
        ) from exc

    try:
        message_type = require_message_type(input_type)
    except (TypeError, ValueError) as exc:
        raise McapConversionError(str(exc)) from exc

    if message_type is MessageType.UNKNOWN:
        raise McapConversionError(f"Unsupported OSI message type: {input_type}")

    reader = SingleTraceReader(enable_message_type_inference=False)
    reader.set_message_type(message_type)
    if not reader.open(source):
        raise McapConversionError(f"Could not open input OSI trace: {source}")

    metadata = prepare_required_file_metadata()
    metadata.update(
        {
            "min_osi_version": OSI_VERSION.as_string(),
            "max_osi_version": OSI_VERSION.as_string(),
            "description": description or f"Converted from {source.name}",
            "data_sources": "CARLA OSI Publisher",
            "zero_time": "0",
        }
    )
    channel_metadata = {
        "net.asam.osi.trace.channel.osi_version": OSI_VERSION.as_string(),
        "net.asam.osi.trace.channel.protobuf_version": protobuf_version,
        "net.asam.osi.trace.channel.description": f"{message_type.value} messages",
    }

    writer = MultiTraceWriter()
    writer_kwargs: dict[str, Any] = {}
    if compression is not None:
        writer_kwargs["compression"] = compression
    if chunk_size is not None:
        writer_kwargs["chunk_size"] = chunk_size
    if not writer.open(target, metadata=metadata, **writer_kwargs):
        reader.close()
        raise McapConversionError(f"Could not open output MCAP file: {target}")

    count = 0
    channel_added = False
    try:
        with reader, writer:
            for result in reader:
                if result.message is None:
                    raise McapConversionError(
                        result.error_message or f"Could not decode message {count} from {source}"
                    )
                if not channel_added:
                    writer.add_channel(topic, type(result.message), metadata=channel_metadata)
                    channel_added = True
                if not writer.write_message(result.message, topic):
                    raise McapConversionError(f"Could not write message {count} to {target}")
                count += 1
    except McapConversionError:
        raise
    except (OSError, RuntimeError, ValueError) as exc:
        raise McapConversionError(f"Failed to convert {source} to {target}: {exc}") from exc

    if count == 0:
        raise McapConversionError(f"Input OSI trace contains no {message_type.value} messages: {source}")

    return McapConversionResult(
        input_path=source,
        output_path=target,
        message_type=message_type.value,
        topic=topic,
        message_count=count,
    )
