"""Command-line entry point."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from .carla import CarlaClient
from .config import GroundTruthConfig, PublisherConfig
from .groundtruth import GroundTruthBuilder
from .mcap import DualMcapWriter, McapConversionError, convert_osi_to_mcap
from .streaming import StreamingUpdateBuilder
from .trace import LengthPrefixedTraceWriter, write_length_prefixed_trace
from .validator import GroundTruthValidator
from .version import OSI_VERSION, __version__


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="carla-osi")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    groundtruth = subparsers.add_parser("groundtruth", help="publish CARLA GroundTruth")
    groundtruth.add_argument("--host", default="127.0.0.1")
    groundtruth.add_argument("--port", type=int, default=2000)
    groundtruth.add_argument("--timeout", type=float, default=10.0)
    groundtruth.add_argument("--ego", default="hero")
    stepping = groundtruth.add_mutually_exclusive_group()
    stepping.add_argument("--sync", action="store_true")
    stepping.add_argument(
        "--wait-for-tick",
        action="store_true",
        help="wait for another CARLA client to advance the world",
    )
    groundtruth.add_argument("--delta-seconds", type=float, default=0.05)
    groundtruth.add_argument("--steps", type=int, default=1)
    groundtruth.add_argument("--duration-seconds", type=float)
    groundtruth.add_argument(
        "--update-mode",
        choices=["groundtruth", "streaming", "dual"],
        default="groundtruth",
        help="GroundTruth, StreamingUpdate, or both channels in one MCAP",
    )
    groundtruth.add_argument("--output", type=Path)
    groundtruth.add_argument("--pretty", action="store_true")
    groundtruth.add_argument("--no-static-objects", action="store_true")
    groundtruth.add_argument("--no-traffic-signs", action="store_true")
    groundtruth.add_argument("--no-traffic-lights", action="store_true")
    groundtruth.add_argument("--no-flip-y", action="store_true")
    groundtruth.add_argument("--require-host-vehicle", action="store_true")

    convert = subparsers.add_parser(
        "convert",
        aliases=["osi-to-mcap", "osi2mcap"],
        help="convert a single-channel OSI trace to MCAP",
    )
    convert.add_argument("input", type=Path, help="input .osi trace")
    convert.add_argument("output", type=Path, help="output .mcap file")
    convert.add_argument(
        "--input-type",
        default="GroundTruth",
        choices=[
            "GroundTruth",
            "SensorData",
            "SensorView",
            "SensorViewConfiguration",
            "HostVehicleData",
            "TrafficCommand",
            "TrafficCommandUpdate",
            "TrafficUpdate",
            "MotionRequest",
            "StreamingUpdate",
        ],
    )
    convert.add_argument("--topic", default="ground_truth")
    convert.add_argument("--compression", choices=["none", "lz4", "zstd"])
    convert.add_argument("--chunk-size", type=int)
    convert.add_argument("--description")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "groundtruth":
        if args.update_mode == "dual":
            return _dual_groundtruth(args, config=None)
        return _groundtruth(args)
    if args.command in {"convert", "osi-to-mcap", "osi2mcap"}:
        return _convert(args)
    return 2


def _groundtruth(args: argparse.Namespace) -> int:
    config = PublisherConfig(
        host=args.host,
        port=args.port,
        timeout_seconds=args.timeout,
        sync=args.sync,
        delta_seconds=args.delta_seconds,
        ground_truth=GroundTruthConfig(
            ego=args.ego,
            flip_y=not args.no_flip_y,
            include_static_objects=not args.no_static_objects,
            include_traffic_signs=not args.no_traffic_signs,
            include_traffic_lights=not args.no_traffic_lights,
        ),
    )
    if args.update_mode == "streaming":
        return _streaming_groundtruth(args, config)

    client = CarlaClient(config)
    builder = GroundTruthBuilder(config.ground_truth)
    validator = GroundTruthValidator()
    messages = []
    try:
        with client.connected() as world:
            builder.carla = client.carla
            for snapshot in _capture_snapshots(client, args, config):
                result = builder.build(world, snapshot=snapshot)
                report = validator.validate(
                    result.message,
                    require_host_vehicle=args.require_host_vehicle,
                )
                if not report.valid:
                    for error in report.errors:
                        print(f"ERROR: {error}", file=sys.stderr)
                    return 1
                messages.append(result.message)
                if args.pretty:
                    print(json.dumps(_summary(result), indent=2))
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if args.output:
        count = write_length_prefixed_trace(args.output, messages)
        if not args.pretty:
            print(f"Wrote {count} GroundTruth messages to {args.output}")
    return 0


def _dual_groundtruth(args: argparse.Namespace, config: PublisherConfig | None = None) -> int:
    if args.output is None:
        print("ERROR: --output is required for dual mode and must be an .mcap file", file=sys.stderr)
        return 1
    if args.output.suffix.lower() != ".mcap":
        print("ERROR: dual mode --output must use the .mcap extension", file=sys.stderr)
        return 1

    config = config or PublisherConfig(
        host=args.host,
        port=args.port,
        timeout_seconds=args.timeout,
        sync=args.sync,
        delta_seconds=args.delta_seconds,
        ground_truth=GroundTruthConfig(
            ego=args.ego,
            flip_y=not args.no_flip_y,
            include_static_objects=not args.no_static_objects,
            include_traffic_signs=not args.no_traffic_signs,
            include_traffic_lights=not args.no_traffic_lights,
        ),
    )
    client = CarlaClient(config)
    ground_truth_builder = GroundTruthBuilder(config.ground_truth)
    streaming_builder = StreamingUpdateBuilder(config.ground_truth)
    validator = GroundTruthValidator()
    try:
        with client.connected() as world, DualMcapWriter(
            args.output,
            description="CARLA 0.9.16 dual GroundTruth and StreamingUpdate capture",
        ) as output_writer:
            ground_truth_builder.carla = client.carla
            streaming_builder.carla = client.carla
            for snapshot in _capture_snapshots(client, args, config):
                ground_truth_result = ground_truth_builder.build(world, snapshot=snapshot)
                streaming_result = streaming_builder.build(world, snapshot=snapshot)
                report = validator.validate(
                    ground_truth_result.message,
                    require_host_vehicle=args.require_host_vehicle,
                )
                if not report.valid:
                    for error in report.errors:
                        print(f"ERROR: {error}", file=sys.stderr)
                    return 1
                output_writer.write(ground_truth_result.message, streaming_result.message)
                if args.pretty:
                    print(
                        json.dumps(
                            {
                                "osi_message": "GroundTruth + StreamingUpdate",
                                "actor_count": ground_truth_result.actor_count,
                                "ground_truth": {
                                    "moving_object_count": ground_truth_result.moving_object_count,
                                    "stationary_object_count": ground_truth_result.stationary_object_count,
                                    "traffic_sign_count": ground_truth_result.traffic_sign_count,
                                    "traffic_light_count": ground_truth_result.traffic_light_count,
                                },
                                "streaming_update": {
                                    "initial": streaming_result.initial,
                                    "moving_object_update_count": streaming_result.moving_object_count,
                                    "stationary_object_update_count": streaming_result.stationary_object_count,
                                    "traffic_sign_update_count": streaming_result.traffic_sign_count,
                                    "traffic_light_update_count": streaming_result.traffic_light_count,
                                    "obsolete_id_count": len(streaming_result.message.obsolete_id),
                                },
                            },
                            indent=2,
                        )
                    )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if not args.pretty:
        print(
            f"Wrote {output_writer.ground_truth_count} GroundTruth and "
            f"{output_writer.streaming_update_count} StreamingUpdate messages "
            f"to {args.output} "
            f"(topics={output_writer.ground_truth_topic},{output_writer.streaming_topic})"
        )
    return 0


def _streaming_groundtruth(args: argparse.Namespace, config: PublisherConfig) -> int:
    client = CarlaClient(config)
    builder = StreamingUpdateBuilder(config.ground_truth)
    validator = GroundTruthValidator()
    output_writer = LengthPrefixedTraceWriter(args.output) if args.output else None
    try:
        with client.connected() as world:
            builder.carla = client.carla
            for snapshot in _capture_snapshots(client, args, config):
                result = builder.build(world, snapshot=snapshot)
                if result.initial:
                    report = validator.validate(
                        result.source_ground_truth,
                        require_host_vehicle=args.require_host_vehicle,
                    )
                    if not report.valid:
                        for error in report.errors:
                            print(f"ERROR: {error}", file=sys.stderr)
                        return 1
                if output_writer:
                    output_writer.write_message(result.message)
                if args.pretty:
                    print(
                        json.dumps(
                            {
                                "osi_message": "StreamingUpdate",
                                "initial": result.initial,
                                "actor_count": result.actor_count,
                                "moving_object_update_count": result.moving_object_count,
                                "stationary_object_update_count": result.stationary_object_count,
                                "traffic_sign_update_count": result.traffic_sign_count,
                                "traffic_light_update_count": result.traffic_light_count,
                                "obsolete_id_count": len(result.message.obsolete_id),
                                "timestamp": {
                                    "seconds": result.message.timestamp.seconds,
                                    "nanos": result.message.timestamp.nanos,
                                },
                            },
                            indent=2,
                        )
                    )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if output_writer:
            output_writer.close()

    if output_writer and not args.pretty:
        print(f"Wrote {output_writer.count} StreamingUpdate messages to {args.output}")
    return 0


def _capture_steps(args: argparse.Namespace, config: PublisherConfig) -> int:
    return max(args.steps, 1)


def _capture_snapshots(client: CarlaClient, args: argparse.Namespace, config: PublisherConfig):
    if args.duration_seconds is None:
        for _ in range(_capture_steps(args, config)):
            yield client.wait_for_tick() if args.wait_for_tick else client.tick()
        return

    if args.duration_seconds <= 0:
        raise ValueError("--duration-seconds must be greater than zero")

    start_seconds: float | None = None
    while True:
        snapshot = client.wait_for_tick() if args.wait_for_tick else client.tick()
        elapsed_seconds = float(snapshot.timestamp.elapsed_seconds)
        if start_seconds is None:
            start_seconds = elapsed_seconds
        yield snapshot
        if elapsed_seconds - start_seconds >= args.duration_seconds:
            return


def _summary(result: Any) -> dict[str, object]:
    return {
        "osi_version": OSI_VERSION.as_string(),
        "actor_count": result.actor_count,
        "moving_object_count": result.moving_object_count,
        "stationary_object_count": result.stationary_object_count,
        "traffic_sign_count": result.traffic_sign_count,
        "traffic_light_count": result.traffic_light_count,
        "timestamp": {
            "seconds": result.message.timestamp.seconds,
            "nanos": result.message.timestamp.nanos,
        },
    }


def _convert(args: argparse.Namespace) -> int:
    try:
        result = convert_osi_to_mcap(
            args.input,
            args.output,
            input_type=args.input_type,
            topic=args.topic,
            compression=args.compression,
            chunk_size=args.chunk_size,
            description=args.description,
        )
    except McapConversionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(
        f"Converted {result.message_count} {result.message_type} messages "
        f"from {result.input_path} to {result.output_path} "
        f"(topic={result.topic})"
    )
    return 0
