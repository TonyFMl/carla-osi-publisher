"""Inspect topics and protobuf types in an OSI MCAP file."""

from __future__ import annotations

import argparse
from pathlib import Path

from osi_utilities import MultiTraceReader


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="MCAP file to inspect")
    args = parser.parse_args()

    reader = MultiTraceReader()
    if not reader.open(args.input):
        raise SystemExit(f"Could not open MCAP file: {args.input}")

    topics = reader.get_available_topics()
    print(f"file: {args.input}")
    print(f"topics: {topics}")
    print(f"file_metadata: {reader.get_file_metadata()}")

    counts: dict[str, int] = {topic: 0 for topic in topics}
    schemas: dict[str, str] = {}
    metadata = {topic: reader.get_channel_metadata(topic) for topic in topics}
    for result in reader:
        counts[result.channel_name] = counts.get(result.channel_name, 0) + 1
        if result.message is not None:
            schemas[result.channel_name] = result.message.DESCRIPTOR.full_name
    reader.close()

    for topic in topics:
        print(f"{topic}: count={counts[topic]} schema={schemas.get(topic, 'unknown')}")
        if metadata[topic] is not None:
            print(f"  metadata: {metadata[topic]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
