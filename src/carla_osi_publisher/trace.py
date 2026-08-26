"""Minimal OSI single-channel trace writer for the GroundTruth MVP."""

from __future__ import annotations

import struct
from collections.abc import Iterable
from pathlib import Path
from typing import Any


class LengthPrefixedTraceWriter:
    """Incremental writer for the OSI single-channel binary trace format."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._stream = self.path.open("wb")
        self.count = 0

    def write_message(self, message: Any) -> None:
        payload = message.SerializeToString()
        self._stream.write(struct.pack("<I", len(payload)))
        self._stream.write(payload)
        self.count += 1

    def close(self) -> None:
        self._stream.close()

    def __enter__(self) -> LengthPrefixedTraceWriter:
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()


def write_length_prefixed_trace(path: str | Path, messages: Iterable[Any]) -> int:
    """Write serialized protobuf messages using the OSI single-channel framing."""

    count = 0
    with Path(path).open("wb") as stream:
        for message in messages:
            payload = message.SerializeToString()
            stream.write(struct.pack("<I", len(payload)))
            stream.write(payload)
            count += 1
    return count
