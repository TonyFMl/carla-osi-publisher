# CARLA OSI Publisher (English)

[中文](README.md)

Python sidecar for converting CARLA 0.9.16 simulation state into ASAM Open
Simulation Interface (OSI) 3.8.0 protobuf messages.

The publisher connects to a running CARLA server through the CARLA Python API.
It does not modify or replace the CARLA server. The conversion layer is kept
independent from the simulator process so that the same architecture can later
support other output sinks, such as UDP, ROS 2, or a live message bus.

The CARLA 0.9.16 server must be downloaded, installed, and started separately
by the user. This repository does not include the CARLA server executable,
maps, or simulation assets. All Python dependencies other than the CARLA
server are installed from PyPI and managed through `uv`.

## What It Does

The current MVP focuses on environment ground truth:

- Convert CARLA vehicles and pedestrians to `osi3.GroundTruth`.
- Convert CARLA environment objects, traffic signs, and traffic lights.
- Select an ego vehicle using its actor ID or `role_name`.
- Produce initial-plus-incremental `osi3.StreamingUpdate` messages.
- Write a complete GroundTruth trace, a StreamingUpdate trace, or both
  channels into one MCAP file.
- Attach OSI file metadata, protobuf descriptors, and channel metadata.
- Validate the GroundTruth fields populated by this project.

The project does not currently spawn sensors, convert camera or lidar data, or
publish live network messages.

### OSI Support Matrix

The current implementation converts CARLA simulation state into only the two
OSI messages below:

| OSI message | CARLA conversion | `.osi` output | `.mcap` output |
| --- | :---: | :---: | :---: |
| `osi3.GroundTruth` | ✅ | ✅ | ✅ dual channel |
| `osi3.StreamingUpdate` | ✅ | ✅ | ✅ dual channel |

Other OSI messages are outside the current project scope.

## Quick Start

The following demo assumes that the CARLA 0.9.16 server has been installed
and started separately and is reachable at `127.0.0.1:2000`.

### 1. Install dependencies

If `uv` is not installed, Linux or WSL can use:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

From the repository root, run this one command to create the virtual
environment and install all Python dependencies:

```bash
uv sync
```

Users do not need to separately download `osi3`, `osi-python`,
`osi_utilities`, or the `asam-osi-utilities` source repository, and do not
need to run a separate `pip install` command. `uv sync` resolves and installs
the Python dependencies from `pyproject.toml` and `uv.lock`, including the
CARLA Python client API, OSI protobuf bindings, and MCAP.

When the publisher runs in WSL, the CARLA Python API is installed in the WSL
environment. Use the Windows host IP instead of `127.0.0.1` when required.

The CARLA server itself is not installed by `uv` or `pip`. Install and start
the CARLA 0.9.16 server separately from the official CARLA distribution.

Check the installation:

```bash
uv run carla-osi --version
```

### 2. Generate traffic

CARLA's official traffic example can be used to create one ego vehicle and
traffic vehicles. The `--hero` option marks one of the requested vehicles as
`role_name=hero`.

```bash
python3 /path/to/Carla/PythonAPI/examples/generate_traffic.py \
  --host 127.0.0.1 \
  --port 2000 \
  --number-of-vehicles 16 \
  --number-of-walkers 0 \
  --hero \
  --seed 42 \
  --tm-port 8000
```

On Windows, replace `python3` with `python` when the `python3` command is not
available.

`--number-of-vehicles 16 --hero` means 15 traffic vehicles plus one ego
vehicle. The official script keeps running and owns the synchronous CARLA
tick. Keep it running while recording.

## Core Usage

After traffic has been generated, run the following commands in a second
terminal. Stop the traffic-generation terminal with `Ctrl+C` after recording.

The command-line entry point is `carla-osi groundtruth`. Use `--update-mode`
to select the OSI message type and the output extension to select the file
format.

### GroundTruth to `.osi`

```bash
uv run carla-osi groundtruth \
  --host 127.0.0.1 \
  --port 2000 \
  --ego hero \
  --sync \
  --duration-seconds 5 \
  --update-mode groundtruth \
  --output ground_truth.osi
```

### StreamingUpdate to `.osi`

```bash
uv run carla-osi groundtruth \
  --host 127.0.0.1 \
  --port 2000 \
  --ego hero \
  --sync \
  --duration-seconds 5 \
  --update-mode streaming \
  --output streaming_update.osi
```

The initial message contains static objects and traffic signs. Later messages
contain moving-object and traffic-light updates. Consumers must retain object
state and apply `obsolete_id`.

### Dual-channel `.mcap`

```bash
uv run carla-osi groundtruth \
  --host 127.0.0.1 \
  --port 2000 \
  --ego hero \
  --sync \
  --duration-seconds 5 \
  --update-mode dual \
  --output capture.mcap
```

The MCAP contains two channels:

| Channel | Message |
| --- | --- |
| `ground_truth` | `osi3.GroundTruth` |
| `streaming_update` | `osi3.StreamingUpdate` |

## OSI MCAP Visualization

Generated OSI MCAP files can be visualized in Lichtblick with the
[asam-osi-converter plugin](https://github.com/Lichtblick-Suite/asam-osi-converter).

![Lichtblick visualization of a CARLA OSI MCAP](docs/images/carla-osi-lichtblick-demo.png)

## Detailed Coverage

### GroundTruth Field Coverage

| OSI area | Status | Current implementation |
| --- | --- | --- |
| `GroundTruth.version` | Supported | Fixed to OSI `3.8.0`. |
| `GroundTruth.timestamp` | Supported | Derived from CARLA snapshot simulation time. |
| `GroundTruth.host_vehicle_id` | Supported | Resolved by numeric actor ID or actor attribute `role_name`. |
| `GroundTruth.moving_object` | Supported | CARLA `vehicle.*` and `walker.pedestrian.*` actors. |
| Moving-object dimensions | Supported | CARLA bounding-box extents converted to full length, width, and height. |
| Moving-object pose | Supported | Position and orientation with configurable lateral-axis conversion. |
| Moving-object velocity | Supported | CARLA actor velocity when exposed by the API. |
| Moving-object acceleration | Supported | CARLA actor acceleration when exposed by the API. |
| Moving-object angular rate | Supported | CARLA angular velocity converted to OSI orientation rate. |
| Vehicle classification | Partial | Uses `osi_vehicle_type` or `object_type` attributes; otherwise `TYPE_UNKNOWN`. |
| Vehicle attributes | Partial | Wheel count, wheel radius, ground clearance, and bounding-box offsets when blueprint attributes exist. |
| Vehicle light state | Partial | Maps CARLA light-state flags to OSI headlight, high-beam, reverse, brake, and indicator states. |
| `stationary_object` | Supported | CARLA environment objects for configured `CityObjectLabel` categories. |
| Stationary classification | Partial | Basic mapping for buildings, barriers, poles, vegetation, walls, and bridges; other labels use `TYPE_OTHER`. |
| `traffic_sign` | Partial | Speed-limit, stop, give-way, and fallback sign mapping from CARLA environment-object names. |
| Traffic-sign value | Partial | Speed-limit numeric value and unit are populated when the name contains a numeric limit. |
| `traffic_light` | Partial | Actor ID, pose, dimensions, color, mode, and icon. |
| Traffic-light geometry | Not implemented | No individual bulb geometry, arrows, lane assignments, or signal groups. |
| Lane network | Not implemented | No complete lanes, lane boundaries, reference lines, or OpenDRIVE topology. |
| Environment conditions | Not implemented | No weather or environmental-condition update. |
| Host vehicle internal data | Not implemented | No standalone `HostVehicleData` output. |
| Sensor detections and raw data | Not implemented | No camera, lidar, radar, or sensor-data payload conversion. |

### StreamingUpdate Coverage

| Field | Status | Behavior |
| --- | --- | --- |
| `version` | Supported | Set to OSI `3.8.0`. |
| `timestamp` | Supported | Copied from the CARLA snapshot used for the update. |
| `stationary_object_update` | Supported | Sent on the initial message only by default. |
| `moving_object_update` | Supported | Sent on the initial message and subsequent updates. |
| `traffic_sign_update` | Supported | Sent on the initial message only by default. |
| `traffic_light_update` | Supported | Sent on the initial message and subsequent updates. |
| `environmental_conditions_update` | Not implemented | No CARLA weather conversion. |
| `host_vehicle_data_update` | Not implemented | No host-internal data conversion. |
| `obsolete_id` | Supported | Reports moving objects and traffic lights that disappear from later snapshots. |

`StreamingUpdate` is a standard OSI top-level message, not a custom extension.
It is an incremental protocol: a consumer must retain the latest object state.
MCAP stores messages and metadata, but does not merge StreamingUpdate messages
itself.

## Validation

`GroundTruthValidator` validates the subset of GroundTruth that this MVP
populates:

- OSI interface version is present and equals `3.8.0`.
- Timestamp is present.
- `host_vehicle_id` is present when `--require-host-vehicle` is used.
- Object IDs are present and unique across supported GroundTruth categories.
- Moving and stationary object base fields are present.
- Traffic-sign and traffic-light base fields are present.
- The host vehicle ID refers to a moving object.

This is a project-level validation layer, not a replacement for a complete
official OSI conformance validator. Missing source data is left unset rather
than fabricated.

## Coordinate System

The default conversion follows the convention used by the existing
`Carla-OSI-Service` implementation:

```text
OSI.x   = CARLA.x
OSI.y   = -CARLA.y
OSI.z   = CARLA.z
OSI.yaw = -CARLA.yaw
```

Use `--no-flip-y` when the consumer expects the native CARLA lateral axis.
The coordinate convention must remain consistent across one trace.

## Identifier Policy

The publisher uses namespaced 64-bit OSI identifiers:

| Source entity | Namespace |
| --- | ---: |
| CARLA actor | 1 |
| CARLA environment object | 2 |
| CARLA traffic light and bulb index | 3 |
| Lane identifiers reserved for future use | 4 |

The namespace prevents actor, environment-object, and traffic-light IDs from
colliding. Oversized CARLA environment IDs are deterministically folded into
the available OSI payload space, with collision resolution within one mapper
instance.

## Architecture

```text
CARLA 0.9.16 server
        |
        | CARLA Python API
        v
CarlaClient
        |
        v
GroundTruthBuilder ------> osi3.GroundTruth
        |
        +-----------------> StreamingUpdateBuilder
        |                           |
        |                           v
        |                   osi3.StreamingUpdate
        v
GroundTruthValidator
        |
        +--> length-prefixed .osi trace
        +--> single-channel MCAP
        +--> dual-channel MCAP
```

Important runtime behavior:

- `--sync` makes the publisher call `world.tick()` and restore the previous
  CARLA world settings on exit.
- `--wait-for-tick` observes snapshots advanced by another CARLA client.
- GroundTruth mode accumulates messages before writing the `.osi` file.
- Streaming mode writes messages incrementally.
- Dual mode writes both messages incrementally to one MCAP file.
- No network publisher is included yet.

## Project Layout

```text
src/
  carla_osi_publisher/
    carla.py       CARLA connection and synchronous/asynchronous stepping
    cli.py         carla-osi command-line interface
    config.py      Publisher and GroundTruth conversion settings
    geometry.py    Coordinate, pose, dimension, and timestamp conversion
    groundtruth.py CARLA world to GroundTruth builder
    ids.py         Namespaced OSI identifier mapping
    mcap.py        Single-channel conversion and dual-channel MCAP writer
    osi.py         Lazy OSI and CARLA dependency loading
    streaming.py   GroundTruth snapshot to StreamingUpdate builder
    trace.py       Length-prefixed single-channel trace writer
    validator.py   MVP GroundTruth validation
    version.py     Project and OSI version constants
examples/
  inspect_mcap.py  MCAP topic, schema, count, and metadata demo
tests/
  fakes.py       CARLA-shaped test doubles
  test_*.py      Conversion, validation, trace, and MCAP tests
pyproject.toml   Package metadata and uv configuration
uv.lock         Reproducible dependency lock file
```

## Known Limitations

- CARLA traffic generation is not part of the publisher CLI. Use CARLA's
  `generate_traffic.py` or another traffic client.
- There is no sensor callback or sensor-spawning implementation.
- Lane topology and full OpenDRIVE semantics are not emitted.
- Traffic signs and traffic lights use basic mappings and do not contain all
  OSI geometry or semantic details.
- `asam-osi-converter` can visualize the `ground_truth` MCAP topic, but its
  current converter does not reconstruct a scene from the
  `streaming_update` topic.
- StreamingUpdate consumers must implement cross-message state retention and
  `obsolete_id` handling.
- The CARLA server and map assets are not distributed with this project and
  must be installed and prepared separately.

## Roadmap

1. Add CARLA traffic-generation helpers with explicit lifecycle management.
2. Add `SensorView` and sensor callback queues for camera, lidar, and radar.
3. Add `SensorViewConfiguration` and sensor spawning.
4. Add `TrafficUpdate` input and external actor-state application.
5. Add lane boundaries, lane topology, and OpenDRIVE references.
6. Add network output sinks such as UDP, ROS 2, and live MCAP streaming.
7. Add native `StreamingUpdate` state reconstruction to visualization clients.

## License

This project is distributed under the Mozilla Public License 2.0 (MPL-2.0).
See [LICENSE](LICENSE) for the complete license text.
