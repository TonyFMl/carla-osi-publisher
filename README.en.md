# CARLA OSI Publisher

[中文](README.md)

A Python converter from CARLA 0.9.16 to ASAM Open Simulation Interface
(OSI) 3.8.0. The current release records `GroundTruth`, `StreamingUpdate`,
and RGB-camera and ray-cast LiDAR `SensorView` messages to `.osi` or `.mcap`.

The CARLA 0.9.16 server, maps, and simulation assets must be installed and
started separately. This project connects to the server without including or
modifying it. All other Python dependencies are installed from PyPI and
managed with `uv`.

## What It Does

### OSI Support Matrix

| OSI message | CARLA conversion | `.osi` | `.mcap` |
| --- | :---: | :---: | :---: |
| `osi3.GroundTruth` | ✅ | ✅ | ✅ |
| `osi3.StreamingUpdate` | ✅ | ✅ | ✅ |
| `osi3.SensorView`: RGB camera | ✅ | ✅ one camera | ✅ one channel per camera |
| `osi3.SensorView`: ray-cast LiDAR | ✅ | ✅ | ✅ separate channel |
| `osi3.SensorViewConfiguration` | ⚠️ embedded in `SensorView` | ❌ standalone | ❌ standalone |
| `osi3.SensorData` | ❌ | ❌ | ❌ |
| `osi3.TrafficUpdate` | ❌ | ❌ | ❌ |

The current implementation also provides:

- Ego selection by actor ID or `role_name`.
- Initial full state followed by incremental `StreamingUpdate` messages.
- OSI, Protobuf, and channel metadata in MCAP files.
- Project-level validation of the implemented GroundTruth fields.
- Single-type `.osi` to `.mcap` conversion through `asam-osi-utilities`.

Radar, Semantic LiDAR, depth cameras, object detection, SensorData, UDP, and
ROS 2 live publishing are not supported.

## Quick Start

### 1. Prepare the CARLA server

Install and start the CARLA 0.9.16 server separately. The default connection
address is `127.0.0.1:2000`.

When the publisher runs in WSL and CARLA runs on Windows, use the Windows host
IP when required

### 2. Install the Python environment

The repository defaults to Python 3.10. After installing `uv`, run this command
from the repository root:

```bash
uv sync
```

This creates `.venv` and installs the CARLA Python API, `osi-python`,
`asam-osi-utilities`, Protobuf, NumPy, and MCAP dependencies. There is no need
to separately download the `osi3`, `osi-python`, or `asam-osi-utilities`
repositories, or to run an additional `pip install`.

Verify the installation:

```bash
uv run carla-osi --version
```

### 3. Minimal demo

The following command creates a `charger_2020` ego vehicle and records five
seconds in synchronous mode:

```bash
uv run carla-osi record \
  --host 127.0.0.1 \
  --port 2000 \
  --sync \
  --duration-seconds 5 \
  --demo-scene \
  --no-static-objects \
  --compression zstd \
  --output carla_osi_demo.mcap
```

The demo includes four RGB cameras facing left, front, right, and rear, plus
one 64-channel LiDAR:

| Channel | Protobuf type | Content |
| --- | --- | --- |
| `ground_truth` | `osi3.GroundTruth` | Current CARLA world state |
| `streaming_update` | `osi3.StreamingUpdate` | Initial full state and later deltas |
| `sensor_view/camera_0` | `osi3.SensorView` | Left RGB camera |
| `sensor_view/camera_1` | `osi3.SensorView` | Front RGB camera |
| `sensor_view/camera_2` | `osi3.SensorView` | Right RGB camera |
| `sensor_view/camera_3` | `osi3.SensorView` | Rear RGB camera |
| `sensor_view/lidar_0` | `osi3.SensorView` | Ray-cast LiDAR |

`--no-static-objects` avoids writing a large set of map objects on every
GroundTruth frame. Remove it when complete static-environment output is
required, at the cost of substantially larger files and longer conversion
times.

## Existing Traffic Scenes

CARLA's official `generate_traffic.py` can create 15 traffic vehicles and one
ego vehicle:

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

The script advances the synchronous CARLA tick by default. Keep it running and
record from another terminal with `--wait-for-tick`:

```bash
uv run carla-osi record \
  --host 127.0.0.1 \
  --port 2000 \
  --ego hero \
  --wait-for-tick \
  --duration-seconds 5 \
  --camera-yaw=-90 \
  --camera-yaw=0 \
  --camera-yaw=90 \
  --camera-yaw=180 \
  --no-static-objects \
  --compression zstd \
  --output traffic_capture.mcap
```

A CARLA world must have only one synchronous master:

- Use `--sync` when the publisher advances the simulation.
- Use `--wait-for-tick` when another client such as `generate_traffic.py`
  advances the simulation.
- Do not run two clients that both call `world.tick()`.

## Output Formats

### `.osi`

An `.osi` file is a single-type Protobuf trace with a four-byte little-endian
length prefix. GroundTruth and StreamingUpdate can be recorded separately:

```bash
# GroundTruth
uv run carla-osi groundtruth \
  --host 127.0.0.1 \
  --port 2000 \
  --ego hero \
  --wait-for-tick \
  --duration-seconds 5 \
  --update-mode groundtruth \
  --output ground_truth.osi

# StreamingUpdate
uv run carla-osi groundtruth \
  --host 127.0.0.1 \
  --port 2000 \
  --ego hero \
  --wait-for-tick \
  --duration-seconds 5 \
  --update-mode streaming \
  --output streaming_update.osi
```

A single camera or LiDAR SensorView can also be written to `.osi`. Use separate
MCAP channels for multi-sensor recordings.

### `.mcap`

`record` writes a multi-channel MCAP directly. An existing single-type `.osi`
trace can be converted to MCAP:

```bash
uv run carla-osi convert \
  ground_truth.osi \
  ground_truth.mcap \
  --input-type GroundTruth \
  --topic ground_truth \
  --compression zstd
```

Inspect MCAP topics, schemas, message counts, and metadata:

```bash
uv run python examples/inspect_mcap.py carla_osi_demo.mcap
```

## OSI MCAP Visualization

OSI MCAP files can be visualized in Lichtblick with the
[asam-osi-converter plugin](https://github.com/Lichtblick-Suite/asam-osi-converter).

![Lichtblick visualization of a CARLA OSI MCAP](docs/images/carla-osi-lichtblick-demo.png)

The plugin uses `GroundTruth.host_vehicle_id` and
`vehicle_attributes.bbcenter_to_rear` to build the `global`, ego rear-axle,
and sensor frame tree. The demo binds GroundTruth to the exact ego actor. When
CARLA provides no axle attributes, the publisher derives a deterministic
approximation from the vehicle bounding-box half-length.

## Detailed Coverage

### GroundTruth

| Area | Status | Current implementation |
| --- | --- | --- |
| Version and timestamp | ✅ | OSI `3.8.0`, CARLA snapshot simulation time |
| Ego reference | ✅ | Actor ID or `role_name` |
| Vehicles and pedestrians | ✅ | Pose, dimensions, velocity, acceleration, and angular rate |
| Vehicle classification | ⚠️ | OSI/CARLA attributes, otherwise `TYPE_UNKNOWN` |
| Vehicle axle positions | ⚠️ | Explicit attributes first, bounding-box approximation otherwise |
| Vehicle lights | ⚠️ | Basic headlight, high-beam, reverse, brake, and indicator mapping |
| Static environment objects | ✅ | Configured `CityObjectLabel` categories |
| Traffic signs | ⚠️ | Speed-limit, stop, give-way, and fallback types |
| Traffic lights | ⚠️ | ID, pose, dimensions, color, mode, and icon |
| Lane/OpenDRIVE topology | ❌ | Not implemented |
| Weather and environmental conditions | ❌ | Not implemented |

### StreamingUpdate

| Field | Status | Behavior |
| --- | --- | --- |
| `stationary_object_update` | ✅ | Initial message only by default |
| `moving_object_update` | ✅ | Every frame |
| `traffic_sign_update` | ✅ | Initial message only by default |
| `traffic_light_update` | ✅ | Every frame |
| `obsolete_id` | ✅ | Reports removed moving objects and traffic lights |
| Environmental conditions and host internals | ❌ | Not implemented |

`StreamingUpdate` is a standard OSI top-level message. Consumers must retain
state across frames and apply `obsolete_id`; MCAP does not merge incremental
updates into a complete scene.

### SensorView

| Area | Status | Current implementation |
| --- | --- | --- |
| RGB image | ✅ | CARLA BGRA converted to `RGB_U8_LIN` |
| Camera calibration | ✅ | Resolution, horizontal/vertical FOV, and mounting pose |
| LiDAR point cloud | ✅ | Direction, round-trip time of flight, and signal strength |
| `sensor_id` / `host_vehicle_id` | ✅ | Dedicated ID namespace and ego reference |
| `global_ground_truth` | ❌ | Not embedded in SensorView |
| Radar, Semantic LiDAR, depth camera | ❌ | Not implemented |
| SensorData detection and tracking | ❌ | Not implemented |

## Coordinates and IDs

The default coordinate conversion is:

```text
OSI.x   = CARLA.x
OSI.y   = -CARLA.y
OSI.z   = CARLA.z
OSI.yaw = -CARLA.yaw
```

Use `--no-flip-y` to preserve CARLA's native Y axis.

OSI IDs reserve the high eight bits as a namespace:

| Entity | Namespace |
| --- | ---: |
| CARLA actor | 1 |
| CARLA environment object | 2 |
| CARLA traffic light | 3 |
| Reserved lane IDs | 4 |
| CARLA sensor | 5 |

## Project Layout

```text
src/carla_osi_publisher/
  carla.py       CARLA connection and synchronization
  cli.py         carla-osi command-line interface
  groundtruth.py GroundTruth conversion
  streaming.py   StreamingUpdate conversion
  sensorview.py  RGB camera and LiDAR SensorView conversion
  mcap.py        OSI MCAP writers and .osi conversion
  trace.py       Length-prefixed .osi writer
  validator.py   Project-level GroundTruth validation
examples/
  inspect_mcap.py
tests/
docs/images/
```

Generated `.osi` and `.mcap` files, virtual environments, build output, and
caches are excluded by `.gitignore`.

## Known Limitations

- The CARLA server, maps, and scenario assets are not distributed with the
  project.
- `record --demo-scene` creates only the ego vehicle, not a complete traffic
  flow.
- Complete static GroundTruth substantially increases CPU use and MCAP size.
- `.osi` is not suitable for multiple independent sensor channels.
- SensorViewConfiguration is embedded only and is not published separately.
- `asam-osi-converter` does not reconstruct a complete 3D scene from
  `streaming_update`.
- No UDP, ROS 2, or other live network publisher is implemented.
- `GroundTruthValidator` is a project-level check, not complete OSI
  conformance certification.

## License

This project is distributed under the Mozilla Public License 2.0 (MPL-2.0).
See [LICENSE](LICENSE) for the complete license text.
