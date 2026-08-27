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
from .mcap import (
    DualMcapWriter,
    McapConversionError,
    SensorViewMcapWriter,
    SupportedMessagesMcapWriter,
    convert_osi_to_mcap,
)
from .sensorview import (
    CameraSensorCapture,
    CameraSensorConfig,
    CameraSensorViewBuilder,
    LidarSensorCapture,
    LidarSensorConfig,
    LidarSensorViewBuilder,
    camera_topic,
    lidar_topic,
)
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

    sensorview = subparsers.add_parser(
        "sensorview",
        help="publish CARLA camera and LiDAR SensorView",
    )
    sensorview.add_argument("--host", default="127.0.0.1")
    sensorview.add_argument("--port", type=int, default=2000)
    sensorview.add_argument("--timeout", type=float, default=10.0)
    sensorview.add_argument("--ego", default="hero")
    sensorview.add_argument("--sync", action="store_true")
    sensorview.add_argument("--wait-for-tick", action="store_true")
    sensorview.add_argument("--delta-seconds", type=float, default=0.05)
    sensorview.add_argument("--steps", type=int, default=1)
    sensorview.add_argument("--duration-seconds", type=float)
    sensorview.add_argument("--output", type=Path, required=True)
    sensorview.add_argument("--pretty", action="store_true")
    sensorview.add_argument("--no-flip-y", action="store_true")
    sensorview.add_argument("--camera-width", type=int, default=400)
    sensorview.add_argument("--camera-height", type=int, default=300)
    sensorview.add_argument("--camera-fov", type=float, default=90.0)
    sensorview.add_argument("--camera-x", type=float, default=0.0)
    sensorview.add_argument("--camera-y", type=float, default=0.0)
    sensorview.add_argument("--camera-z", type=float, default=2.4)
    sensorview.add_argument("--camera-roll", type=float, default=0.0)
    sensorview.add_argument("--camera-pitch", type=float, default=0.0)
    sensorview.add_argument("--camera-yaw", type=float, action="append")
    sensorview.add_argument("--camera-sensor-tick", type=float)
    sensorview.add_argument("--no-camera", action="store_true")
    sensorview.add_argument("--lidar", action="store_true")
    sensorview.add_argument("--lidar-name", default="lidar_0")
    sensorview.add_argument("--lidar-channels", type=int, default=64)
    sensorview.add_argument("--lidar-range", type=float, default=100.0)
    sensorview.add_argument("--lidar-points-per-second", type=int, default=250000)
    sensorview.add_argument("--lidar-rotation-frequency", type=float, default=20.0)
    sensorview.add_argument("--lidar-upper-fov", type=float, default=10.0)
    sensorview.add_argument("--lidar-lower-fov", type=float, default=-30.0)
    sensorview.add_argument("--lidar-x", type=float, default=0.0)
    sensorview.add_argument("--lidar-y", type=float, default=0.0)
    sensorview.add_argument("--lidar-z", type=float, default=2.4)
    sensorview.add_argument("--lidar-roll", type=float, default=0.0)
    sensorview.add_argument("--lidar-pitch", type=float, default=0.0)
    sensorview.add_argument("--lidar-yaw", type=float, default=0.0)
    sensorview.add_argument("--lidar-sensor-tick", type=float)
    sensorview.add_argument(
        "--demo-scene",
        action="store_true",
        help="spawn a charger_2020 ego and use the four camera directions from visualize_multiple_sensors.py",
    )

    record = subparsers.add_parser(
        "record",
        help="record all currently supported OSI messages to one MCAP",
    )
    record.add_argument("--host", default="127.0.0.1")
    record.add_argument("--port", type=int, default=2000)
    record.add_argument("--timeout", type=float, default=10.0)
    record.add_argument("--ego", default="hero")
    record_stepping = record.add_mutually_exclusive_group()
    record_stepping.add_argument("--sync", action="store_true")
    record_stepping.add_argument("--wait-for-tick", action="store_true")
    record.add_argument("--delta-seconds", type=float, default=0.05)
    record.add_argument("--steps", type=int, default=1)
    record.add_argument("--duration-seconds", type=float)
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--compression", choices=["none", "lz4", "zstd"])
    record.add_argument("--chunk-size", type=int)
    record.add_argument("--pretty", action="store_true")
    record.add_argument("--no-flip-y", action="store_true")
    record.add_argument("--no-static-objects", action="store_true")
    record.add_argument("--no-traffic-signs", action="store_true")
    record.add_argument("--no-traffic-lights", action="store_true")
    record.add_argument("--require-host-vehicle", action="store_true")
    record.add_argument("--camera-width", type=int, default=400)
    record.add_argument("--camera-height", type=int, default=300)
    record.add_argument("--camera-fov", type=float, default=90.0)
    record.add_argument("--camera-x", type=float, default=0.0)
    record.add_argument("--camera-y", type=float, default=0.0)
    record.add_argument("--camera-z", type=float, default=2.4)
    record.add_argument("--camera-roll", type=float, default=0.0)
    record.add_argument("--camera-pitch", type=float, default=0.0)
    record.add_argument("--camera-yaw", type=float, action="append")
    record.add_argument("--camera-sensor-tick", type=float)
    record.add_argument("--no-camera", action="store_true")
    record.add_argument("--no-lidar", action="store_true")
    record.add_argument("--lidar-name", default="lidar_0")
    record.add_argument("--lidar-channels", type=int, default=64)
    record.add_argument("--lidar-range", type=float, default=100.0)
    record.add_argument("--lidar-points-per-second", type=int, default=250000)
    record.add_argument("--lidar-rotation-frequency", type=float, default=20.0)
    record.add_argument("--lidar-upper-fov", type=float, default=10.0)
    record.add_argument("--lidar-lower-fov", type=float, default=-30.0)
    record.add_argument("--lidar-x", type=float, default=0.0)
    record.add_argument("--lidar-y", type=float, default=0.0)
    record.add_argument("--lidar-z", type=float, default=2.4)
    record.add_argument("--lidar-roll", type=float, default=0.0)
    record.add_argument("--lidar-pitch", type=float, default=0.0)
    record.add_argument("--lidar-yaw", type=float, default=0.0)
    record.add_argument("--lidar-sensor-tick", type=float)
    record.add_argument("--demo-scene", action="store_true")

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
    if args.command == "sensorview":
        return _sensorview(args)
    if args.command == "record":
        return _record(args)
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
            for snapshot in _capture_snapshots(client, args):
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


def _sensorview(args: argparse.Namespace) -> int:
    if not args.no_camera and (
        args.camera_width <= 0 or args.camera_height <= 0
    ):
        print("ERROR: camera dimensions must be greater than zero", file=sys.stderr)
        return 1
    if not args.no_camera and (args.camera_fov <= 0 or args.camera_fov >= 180):
        print("ERROR: camera FOV must be between 0 and 180 degrees", file=sys.stderr)
        return 1
    if args.no_camera and not args.lidar:
        print("ERROR: enable a camera or LiDAR sensor", file=sys.stderr)
        return 1
    if args.lidar and (
        args.lidar_channels <= 0
        or args.lidar_range <= 0
        or args.lidar_points_per_second <= 0
        or args.lidar_rotation_frequency <= 0
        or args.lidar_upper_fov <= args.lidar_lower_fov
    ):
        print("ERROR: invalid LiDAR configuration", file=sys.stderr)
        return 1
    if args.sync and args.wait_for_tick:
        print("ERROR: --sync and --wait-for-tick are mutually exclusive", file=sys.stderr)
        return 1

    yaws = [] if args.no_camera else args.camera_yaw
    if not args.no_camera and not yaws:
        yaws = [-90.0, 0.0, 90.0, 180.0] if args.demo_scene else [0.0]
    camera_count = 0 if args.no_camera else len(yaws)
    if args.output.suffix.lower() == ".osi" and camera_count > 1:
        print(
            "ERROR: .osi output supports exactly one camera; "
            "use .mcap for multiple cameras",
            file=sys.stderr,
        )
        return 1
    if args.output.suffix.lower() not in {".osi", ".mcap"}:
        print("ERROR: SensorView output must use .osi or .mcap", file=sys.stderr)
        return 1

    config = PublisherConfig(
        host=args.host,
        port=args.port,
        timeout_seconds=args.timeout,
        sync=args.sync,
        delta_seconds=args.delta_seconds,
    )
    camera_configs = [
        CameraSensorConfig(
            name="camera_front" if camera_count == 1 else f"camera_{index}",
            width=args.camera_width,
            height=args.camera_height,
            fov_degrees=args.camera_fov,
            x=args.camera_x,
            y=args.camera_y,
            z=args.camera_z,
            roll=args.camera_roll,
            pitch=args.camera_pitch,
            yaw=yaw,
            sensor_tick=args.camera_sensor_tick,
        )
        for index, yaw in enumerate(yaws)
    ]
    lidar_config = (
        LidarSensorConfig(
            name=args.lidar_name,
            channels=args.lidar_channels,
            lidar_range=args.lidar_range,
            points_per_second=args.lidar_points_per_second,
            rotation_frequency=args.lidar_rotation_frequency,
            upper_fov_degrees=args.lidar_upper_fov,
            lower_fov_degrees=args.lidar_lower_fov,
            x=args.lidar_x,
            y=args.lidar_y,
            z=args.lidar_z,
            roll=args.lidar_roll,
            pitch=args.lidar_pitch,
            yaw=args.lidar_yaw,
            sensor_tick=args.lidar_sensor_tick,
        )
        if args.lidar
        else None
    )

    client = CarlaClient(config)
    captures: list[CameraSensorCapture] = []
    lidar_capture: LidarSensorCapture | None = None
    output_writer: Any | None = None
    spawned_vehicle: Any | None = None
    traffic_manager: Any | None = None
    try:
        with client.connected() as world:
            try:
                if args.demo_scene:
                    try:
                        traffic_manager = client.client.get_trafficmanager(8000)
                        if args.sync:
                            traffic_manager.set_synchronous_mode(True)
                        blueprint = world.get_blueprint_library().filter("charger_2020")[0]
                        _set_ego_role_name(blueprint)
                        spawn_points = world.get_map().get_spawn_points()
                        if not spawn_points:
                            raise RuntimeError("CARLA map has no vehicle spawn points")
                        spawned_vehicle = world.spawn_actor(
                            blueprint,
                            spawn_points[0],
                        )
                        spawned_vehicle.set_autopilot(
                            True,
                            traffic_manager.get_port(),
                        )
                        host_vehicle = spawned_vehicle
                    except (AttributeError, IndexError) as exc:
                        raise RuntimeError(
                            "Could not create the visualize_multiple_sensors demo scene"
                        ) from exc
                else:
                    host_vehicle = _find_ego_vehicle(world, args.ego)
                    if host_vehicle is None:
                        raise RuntimeError(
                            f"Could not find ego vehicle '{args.ego}'. "
                            "Use --demo-scene or start a vehicle with role_name=hero."
                        )

                for camera_config in camera_configs:
                    captures.append(
                        CameraSensorCapture(
                            world,
                            client.carla,
                            host_vehicle,
                            camera_config,
                            client=client.client,
                        )
                    )
                if lidar_config is not None:
                    lidar_capture = LidarSensorCapture(
                        world,
                        client.carla,
                        host_vehicle,
                        lidar_config,
                        client=client.client,
                    )

                builders = [
                    CameraSensorViewBuilder(
                        camera_config,
                        flip_y=not args.no_flip_y,
                    )
                    for camera_config in camera_configs
                ]
                lidar_builder = (
                    LidarSensorViewBuilder(
                        lidar_config,
                        flip_y=not args.no_flip_y,
                    )
                    if lidar_config is not None
                    else None
                )
                if args.output.suffix.lower() == ".osi":
                    output_writer = LengthPrefixedTraceWriter(args.output)
                else:
                    output_writer = SensorViewMcapWriter(args.output)
                    output_writer.open()

                for snapshot in _capture_snapshots(client, args):
                    frame = int(snapshot.frame)
                    for capture, builder, camera_config in zip(
                        captures,
                        builders,
                        camera_configs,
                    ):
                        image = capture.get_for_frame(frame, args.timeout)
                        message = builder.build(image, capture.sensor, host_vehicle)
                        if isinstance(output_writer, LengthPrefixedTraceWriter):
                            output_writer.write_message(message)
                        else:
                            output_writer.write(message, camera_topic(camera_config))
                        if args.pretty:
                            print(
                                json.dumps(
                                    {
                                        "osi_message": "SensorView",
                                        "sensor": camera_config.name,
                                        "frame": frame,
                                        "image": {
                                            "width": image.width,
                                            "height": image.height,
                                            "bytes": len(message.camera_sensor_view[0].image_data),
                                        },
                                        "timestamp": {
                                            "seconds": message.timestamp.seconds,
                                            "nanos": message.timestamp.nanos,
                                        },
                                    },
                                    indent=2,
                                )
                            )
                    if (
                        lidar_capture is not None
                        and lidar_builder is not None
                        and lidar_config is not None
                    ):
                        measurement = lidar_capture.get_for_frame(frame, args.timeout)
                        message = lidar_builder.build(
                            measurement,
                            lidar_capture.sensor,
                            host_vehicle,
                        )
                        if isinstance(output_writer, LengthPrefixedTraceWriter):
                            output_writer.write_message(message)
                        else:
                            output_writer.write(message, lidar_topic(lidar_config))
                        if args.pretty:
                            print(
                                json.dumps(
                                    {
                                        "osi_message": "SensorView",
                                        "sensor": lidar_config.name,
                                        "frame": frame,
                                        "point_count": len(
                                            message.lidar_sensor_view[0].reflection
                                        ),
                                    },
                                    indent=2,
                                )
                            )
            finally:
                if lidar_capture is not None:
                    try:
                        lidar_capture.destroy()
                    except Exception:
                        pass
                for capture in reversed(captures):
                    try:
                        capture.destroy()
                    except Exception:
                        pass
                if output_writer is not None:
                    output_writer.close()
                if spawned_vehicle is not None:
                    try:
                        client.client.apply_batch(
                            [client.carla.command.DestroyActor(spawned_vehicle)]
                        )
                    except Exception:
                        pass
                if traffic_manager is not None and args.sync:
                    try:
                        traffic_manager.set_synchronous_mode(False)
                    except Exception:
                        pass
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if not args.pretty:
        count = (
            output_writer.count
            if isinstance(output_writer, LengthPrefixedTraceWriter)
            else output_writer.message_count
        )
        print(f"Wrote {count} SensorView messages to {args.output}")
    return 0


def _record(args: argparse.Namespace) -> int:
    if args.output.suffix.lower() != ".mcap":
        print("ERROR: record output must use the .mcap extension", file=sys.stderr)
        return 1
    if args.no_camera and args.no_lidar:
        print("ERROR: record must enable a camera or LiDAR", file=sys.stderr)
        return 1
    if not args.no_camera and (
        args.camera_width <= 0
        or args.camera_height <= 0
        or args.camera_fov <= 0
        or args.camera_fov >= 180
    ):
        print("ERROR: invalid camera configuration", file=sys.stderr)
        return 1
    if not args.no_lidar and (
        args.lidar_channels <= 0
        or args.lidar_range <= 0
        or args.lidar_points_per_second <= 0
        or args.lidar_rotation_frequency <= 0
        or args.lidar_upper_fov <= args.lidar_lower_fov
    ):
        print("ERROR: invalid LiDAR configuration", file=sys.stderr)
        return 1

    camera_yaws = [] if args.no_camera else args.camera_yaw
    if not args.no_camera and not camera_yaws:
        camera_yaws = (
            [-90.0, 0.0, 90.0, 180.0]
            if args.demo_scene
            else [0.0]
        )
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
    camera_configs = [
        CameraSensorConfig(
            name="camera_front" if len(camera_yaws) == 1 else f"camera_{index}",
            width=args.camera_width,
            height=args.camera_height,
            fov_degrees=args.camera_fov,
            x=args.camera_x,
            y=args.camera_y,
            z=args.camera_z,
            roll=args.camera_roll,
            pitch=args.camera_pitch,
            yaw=yaw,
            sensor_tick=args.camera_sensor_tick,
        )
        for index, yaw in enumerate(camera_yaws)
    ]
    lidar_config = (
        LidarSensorConfig(
            name=args.lidar_name,
            channels=args.lidar_channels,
            lidar_range=args.lidar_range,
            points_per_second=args.lidar_points_per_second,
            rotation_frequency=args.lidar_rotation_frequency,
            upper_fov_degrees=args.lidar_upper_fov,
            lower_fov_degrees=args.lidar_lower_fov,
            x=args.lidar_x,
            y=args.lidar_y,
            z=args.lidar_z,
            roll=args.lidar_roll,
            pitch=args.lidar_pitch,
            yaw=args.lidar_yaw,
            sensor_tick=args.lidar_sensor_tick,
        )
        if not args.no_lidar
        else None
    )

    client = CarlaClient(config)
    ground_truth_builder = GroundTruthBuilder(config.ground_truth)
    streaming_builder = StreamingUpdateBuilder(config.ground_truth)
    validator = GroundTruthValidator()
    captures: list[CameraSensorCapture] = []
    lidar_capture: LidarSensorCapture | None = None
    output_writer: SupportedMessagesMcapWriter | None = None
    spawned_vehicle: Any | None = None
    traffic_manager: Any | None = None
    try:
        with client.connected() as world:
            try:
                if args.demo_scene:
                    traffic_manager = client.client.get_trafficmanager(8000)
                    if args.sync:
                        traffic_manager.set_synchronous_mode(True)
                    blueprint = world.get_blueprint_library().filter("charger_2020")[0]
                    _set_ego_role_name(blueprint)
                    spawn_points = world.get_map().get_spawn_points()
                    if not spawn_points:
                        raise RuntimeError("CARLA map has no vehicle spawn points")
                    spawned_vehicle = world.spawn_actor(blueprint, spawn_points[0])
                    spawned_vehicle.set_autopilot(
                        True,
                        traffic_manager.get_port(),
                    )
                    # Bind GroundTruth and StreamingUpdate to the exact demo
                    # vehicle instead of whichever existing actor is named
                    # "hero" first.
                    config.ground_truth.ego = int(spawned_vehicle.id)
                    host_vehicle = spawned_vehicle
                else:
                    host_vehicle = _find_ego_vehicle(world, args.ego)
                    if host_vehicle is None:
                        raise RuntimeError(
                            f"Could not find ego vehicle '{args.ego}'. "
                            "Use --demo-scene or start a vehicle with role_name=hero."
                        )

                ground_truth_builder.carla = client.carla
                streaming_builder.carla = client.carla
                for camera_config in camera_configs:
                    captures.append(
                        CameraSensorCapture(
                            world,
                            client.carla,
                            host_vehicle,
                            camera_config,
                            client=client.client,
                        )
                    )
                if lidar_config is not None:
                    lidar_capture = LidarSensorCapture(
                        world,
                        client.carla,
                        host_vehicle,
                        lidar_config,
                        client=client.client,
                    )

                camera_builders = [
                    CameraSensorViewBuilder(
                        camera_config,
                        flip_y=not args.no_flip_y,
                    )
                    for camera_config in camera_configs
                ]
                lidar_builder = (
                    LidarSensorViewBuilder(
                        lidar_config,
                        flip_y=not args.no_flip_y,
                    )
                    if lidar_config is not None
                    else None
                )
                output_writer = SupportedMessagesMcapWriter(
                    args.output,
                    compression=args.compression,
                    chunk_size=args.chunk_size,
                )
                output_writer.open()

                for snapshot in _capture_snapshots(client, args):
                    frame = int(snapshot.frame)
                    ground_truth_result = ground_truth_builder.build(
                        world,
                        snapshot=snapshot,
                    )
                    streaming_result = streaming_builder.build(
                        world,
                        snapshot=snapshot,
                    )
                    report = validator.validate(
                        ground_truth_result.message,
                        require_host_vehicle=args.require_host_vehicle,
                    )
                    if not report.valid:
                        for error in report.errors:
                            print(f"ERROR: {error}", file=sys.stderr)
                        return 1

                    output_writer.write(
                        ground_truth_result.message,
                        "ground_truth",
                        description="GroundTruth messages",
                    )
                    output_writer.write(
                        streaming_result.message,
                        "streaming_update",
                        description="StreamingUpdate messages",
                    )

                    camera_point_count = 0
                    for capture, builder, camera_config in zip(
                        captures,
                        camera_builders,
                        camera_configs,
                    ):
                        image = capture.get_for_frame(frame, args.timeout)
                        camera_message = builder.build(
                            image,
                            capture.sensor,
                            host_vehicle,
                        )
                        output_writer.write(
                            camera_message,
                            camera_topic(camera_config),
                            description=f"Camera SensorView messages for {camera_config.name}",
                        )
                        camera_point_count += len(
                            camera_message.camera_sensor_view[0].image_data
                        )

                    lidar_point_count = 0
                    if (
                        lidar_capture is not None
                        and lidar_builder is not None
                        and lidar_config is not None
                    ):
                        measurement = lidar_capture.get_for_frame(frame, args.timeout)
                        lidar_message = lidar_builder.build(
                            measurement,
                            lidar_capture.sensor,
                            host_vehicle,
                        )
                        output_writer.write(
                            lidar_message,
                            lidar_topic(lidar_config),
                            description=f"LiDAR SensorView messages for {lidar_config.name}",
                        )
                        lidar_point_count = len(
                            lidar_message.lidar_sensor_view[0].reflection
                        )

                    if args.pretty:
                        print(
                            json.dumps(
                                {
                                    "frame": frame,
                                    "ground_truth": {
                                        "moving_objects": ground_truth_result.moving_object_count,
                                        "stationary_objects": ground_truth_result.stationary_object_count,
                                    },
                                    "streaming_update": {
                                        "initial": streaming_result.initial,
                                        "moving_object_updates": streaming_result.moving_object_count,
                                    },
                                    "sensor_view": {
                                        "camera_bytes": camera_point_count,
                                        "lidar_points": lidar_point_count,
                                    },
                                }
                            )
                        )
            finally:
                if lidar_capture is not None:
                    try:
                        lidar_capture.destroy()
                    except Exception:
                        pass
                for capture in reversed(captures):
                    try:
                        capture.destroy()
                    except Exception:
                        pass
                if output_writer is not None:
                    output_writer.close()
                if spawned_vehicle is not None:
                    try:
                        client.client.apply_batch(
                            [client.carla.command.DestroyActor(spawned_vehicle)]
                        )
                    except Exception:
                        pass
                if traffic_manager is not None and args.sync:
                    try:
                        traffic_manager.set_synchronous_mode(False)
                    except Exception:
                        pass
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if output_writer is not None and not args.pretty:
        print(f"Wrote {output_writer.message_count} messages to {args.output}")
        for topic, count in output_writer.message_counts.items():
            print(f"  {topic}: {count}")
    return 0


def _find_ego_vehicle(world: Any, requested: str | int) -> Any | None:
    for actor in world.get_actors():
        if not str(getattr(actor, "type_id", "")).startswith("vehicle."):
            continue
        actor_id = int(actor.id)
        if isinstance(requested, int) and actor_id == requested:
            return actor
        if isinstance(requested, str) and requested.isdigit() and actor_id == int(requested):
            return actor
        for attribute in getattr(actor, "attributes", []) or []:
            if isinstance(attribute, tuple) and len(attribute) == 2:
                attribute_id, attribute_value = attribute
            else:
                attribute_id = getattr(attribute, "id", "")
                attribute_value = getattr(attribute, "value", "")
            if str(attribute_id) == "role_name" and str(attribute_value) == str(requested):
                return actor
    return None


def _set_ego_role_name(blueprint: Any) -> None:
    """Mark a demo vehicle so GroundTruth can resolve it as the host vehicle."""

    has_attribute = getattr(blueprint, "has_attribute", None)
    if callable(has_attribute) and not has_attribute("role_name"):
        return
    blueprint.set_attribute("role_name", "hero")


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
            for snapshot in _capture_snapshots(client, args):
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
            for snapshot in _capture_snapshots(client, args):
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


def _capture_steps(args: argparse.Namespace) -> int:
    return max(args.steps, 1)


def _capture_snapshots(client: CarlaClient, args: argparse.Namespace):
    if args.duration_seconds is None:
        for _ in range(_capture_steps(args)):
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
