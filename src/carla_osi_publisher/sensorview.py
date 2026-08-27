"""CARLA camera and LiDAR to OSI SensorView conversion helpers."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from queue import Empty, Full, Queue
from typing import Any

from .geometry import mounting_position, timestamp
from .ids import IdMapper
from .osi import osi_message, set_interface_version

_SPEED_OF_LIGHT_M_PER_S = 299_792_458.0


@dataclass(frozen=True, slots=True)
class CameraSensorConfig:
    """Configuration for one CARLA RGB camera."""

    name: str
    width: int = 400
    height: int = 300
    fov_degrees: float = 90.0
    x: float = 0.0
    y: float = 0.0
    z: float = 2.4
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    sensor_tick: float | None = None

    @property
    def vertical_fov_radians(self) -> float:
        horizontal = math.radians(self.fov_degrees)
        return 2.0 * math.atan(
            math.tan(horizontal / 2.0) * self.height / self.width
        )


class CameraSensorViewBuilder:
    """Build OSI SensorView messages from CARLA RGB camera frames."""

    def __init__(
        self,
        config: CameraSensorConfig,
        *,
        flip_y: bool = True,
        id_mapper: IdMapper | None = None,
    ) -> None:
        self.config = config
        self.flip_y = flip_y
        self.ids = id_mapper or IdMapper()

    def build(self, image: Any, sensor_actor: Any, host_vehicle: Any) -> Any:
        """Convert one CARLA camera image into an OSI SensorView."""

        message_class = osi_message("osi_sensorview_pb2", "SensorView")
        message = message_class()
        set_interface_version(message)
        timestamp(message.timestamp, float(image.timestamp))

        sensor_id = self.ids.sensor(int(sensor_actor.id))
        message.sensor_id.value = sensor_id
        message.host_vehicle_id.value = self.ids.actor(int(host_vehicle.id))
        mounting_position(
            message.mounting_position,
            self._carla_transform(),
            flip_y=self.flip_y,
        )

        camera_view = message.camera_sensor_view.add()
        camera_config = camera_view.view_configuration
        camera_config.sensor_id.value = sensor_id
        mounting_position(
            camera_config.mounting_position,
            self._carla_transform(),
            flip_y=self.flip_y,
        )
        camera_config.field_of_view_horizontal = math.radians(self.config.fov_degrees)
        camera_config.field_of_view_vertical = self.config.vertical_fov_radians
        camera_config.number_of_pixels_horizontal = int(image.width)
        camera_config.number_of_pixels_vertical = int(image.height)
        camera_config.channel_format.append(
            camera_config.CHANNEL_FORMAT_RGB_U8_LIN
        )
        camera_config.samples_per_pixel = 3
        camera_config.pixel_order = camera_config.PIXEL_ORDER_DEFAULT
        camera_view.image_data = _carla_bgra_to_rgb(image)
        return message

    def _carla_transform(self) -> Any:
        class Location:
            def __init__(self, x: float, y: float, z: float) -> None:
                self.x, self.y, self.z = x, y, z

        class Rotation:
            def __init__(self, roll: float, pitch: float, yaw: float) -> None:
                self.roll, self.pitch, self.yaw = roll, pitch, yaw

        class Transform:
            def __init__(self, location: Any, rotation: Any) -> None:
                self.location, self.rotation = location, rotation

        return Transform(
            Location(self.config.x, self.config.y, self.config.z),
            Rotation(self.config.roll, self.config.pitch, self.config.yaw),
        )


def _carla_bgra_to_rgb(image: Any) -> bytes:
    """Convert CARLA's four-channel BGRA image buffer to RGB bytes."""

    try:
        import numpy as np
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "RGB SensorView conversion requires numpy. Run `uv sync`."
        ) from exc

    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    expected = int(image.width) * int(image.height) * 4
    if array.size != expected:
        raise ValueError(
            f"Unexpected CARLA image buffer size: expected {expected}, got {array.size}"
        )
    array = array.reshape((int(image.height), int(image.width), 4))
    return np.ascontiguousarray(array[:, :, :3][:, :, ::-1]).tobytes()


class CameraSensorCapture:
    """Own one CARLA RGB sensor and synchronize callbacks by frame number."""

    def __init__(
        self,
        world: Any,
        carla_module: Any,
        host_vehicle: Any,
        config: CameraSensorConfig,
        *,
        client: Any | None = None,
        queue_size: int = 4,
    ) -> None:
        self.config = config
        self._carla_module = carla_module
        self._client = client
        self._queue: Queue[Any] = Queue(maxsize=queue_size)
        self._pending: dict[int, Any] = {}
        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(config.width))
        blueprint.set_attribute("image_size_y", str(config.height))
        blueprint.set_attribute("fov", str(config.fov_degrees))
        if config.sensor_tick is not None:
            blueprint.set_attribute("sensor_tick", str(config.sensor_tick))

        transform = carla_module.Transform(
            carla_module.Location(x=config.x, y=config.y, z=config.z),
            carla_module.Rotation(
                roll=config.roll,
                pitch=config.pitch,
                yaw=config.yaw,
            ),
        )
        self.sensor = world.spawn_actor(blueprint, transform, attach_to=host_vehicle)
        self.sensor.listen(self._on_image)

    def _on_image(self, image: Any) -> None:
        try:
            self._queue.put_nowait(image)
        except Full:
            # Dropping an old callback is preferable to blocking CARLA's
            # sensor callback thread when the consumer is slower than the sensor.
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(image)
            except Empty:
                return

    def get_for_frame(self, frame: int, timeout: float) -> Any:
        """Return the image for ``frame``, discarding stale callbacks."""

        if frame in self._pending:
            return self._pending.pop(frame)

        while True:
            try:
                image = self._queue.get(timeout=timeout)
            except Empty as exc:
                raise TimeoutError(
                    f"Timed out waiting for camera '{self.config.name}' frame {frame}"
                ) from exc
            image_frame = int(image.frame)
            if image_frame == frame:
                return image
            if image_frame > frame:
                self._pending[image_frame] = image
                raise TimeoutError(
                    f"Camera '{self.config.name}' skipped requested frame {frame}; "
                    f"received frame {image_frame}"
                )

    def destroy(self) -> None:
        self.sensor.stop()
        if self._client is not None:
            self._client.apply_batch(
                [self._carla_module.command.DestroyActor(self.sensor)]
            )
        else:
            self.sensor.destroy()


def camera_topic(config: CameraSensorConfig) -> str:
    return f"sensor_view/{config.name}"


@dataclass(frozen=True, slots=True)
class LidarSensorConfig:
    """Configuration for one CARLA ray-cast LiDAR."""

    name: str
    channels: int = 64
    lidar_range: float = 100.0
    points_per_second: int = 250_000
    rotation_frequency: float = 20.0
    upper_fov_degrees: float = 10.0
    lower_fov_degrees: float = -30.0
    x: float = 0.0
    y: float = 0.0
    z: float = 2.4
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    sensor_tick: float | None = None

    @property
    def vertical_fov_radians(self) -> float:
        return math.radians(self.upper_fov_degrees - self.lower_fov_degrees)

    @property
    def horizontal_rays(self) -> int:
        if self.channels <= 0 or self.rotation_frequency <= 0:
            return 1
        return max(
            1,
            round(
                self.points_per_second
                / self.rotation_frequency
                / self.channels
            ),
        )


class LidarSensorViewBuilder:
    """Build OSI SensorView messages from CARLA ray-cast LiDAR frames."""

    def __init__(
        self,
        config: LidarSensorConfig,
        *,
        flip_y: bool = True,
        id_mapper: IdMapper | None = None,
    ) -> None:
        self.config = config
        self.flip_y = flip_y
        self.ids = id_mapper or IdMapper()

    def build(self, measurement: Any, sensor_actor: Any, host_vehicle: Any) -> Any:
        """Convert one CARLA LiDAR measurement into an OSI SensorView."""

        message_class = osi_message("osi_sensorview_pb2", "SensorView")
        message = message_class()
        set_interface_version(message)
        timestamp(message.timestamp, float(measurement.timestamp))

        sensor_id = self.ids.sensor(int(sensor_actor.id))
        message.sensor_id.value = sensor_id
        message.host_vehicle_id.value = self.ids.actor(int(host_vehicle.id))
        mounting_position(
            message.mounting_position,
            self._carla_transform(),
            flip_y=self.flip_y,
        )

        lidar_view = message.lidar_sensor_view.add()
        lidar_config = lidar_view.view_configuration
        lidar_config.sensor_id.value = sensor_id
        mounting_position(
            lidar_config.mounting_position,
            self._carla_transform(),
            flip_y=self.flip_y,
        )
        lidar_config.field_of_view_horizontal = 2.0 * math.pi
        lidar_config.field_of_view_vertical = self.config.vertical_fov_radians
        lidar_config.number_of_rays_horizontal = self.config.horizontal_rays
        lidar_config.number_of_rays_vertical = self.config.channels
        lidar_config.max_number_of_interactions = 1
        lidar_config.num_of_pixels = 0

        for x, y, z, intensity in _carla_lidar_points(measurement):
            distance = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(distance) or distance <= 0.0:
                continue

            direction = lidar_config.directions.add()
            direction.x = x / distance
            direction.y = (-y if self.flip_y else y) / distance
            direction.z = z / distance

            reflection = lidar_view.reflection.add()
            # CARLA gives the one-way hit distance. OSI models the travel time
            # of the emitted and reflected ray, so use the round-trip time.
            reflection.time_of_flight = (
                2.0 * distance / _SPEED_OF_LIGHT_M_PER_S
            )
            reflection.signal_strength = float(intensity)
            lidar_config.num_of_pixels += 1

        return message

    def _carla_transform(self) -> Any:
        class Location:
            def __init__(self, x: float, y: float, z: float) -> None:
                self.x, self.y, self.z = x, y, z

        class Rotation:
            def __init__(self, roll: float, pitch: float, yaw: float) -> None:
                self.roll, self.pitch, self.yaw = roll, pitch, yaw

        class Transform:
            def __init__(self, location: Any, rotation: Any) -> None:
                self.location, self.rotation = location, rotation

        return Transform(
            Location(self.config.x, self.config.y, self.config.z),
            Rotation(self.config.roll, self.config.pitch, self.config.yaw),
        )


def _carla_lidar_points(measurement: Any) -> list[tuple[float, float, float, float]]:
    """Read CARLA's float32 ``x, y, z, intensity`` point buffer."""

    raw_data = measurement.raw_data
    if len(raw_data) % 16 != 0:
        raise ValueError(
            "Unexpected CARLA LiDAR buffer size: expected a multiple of 16 bytes, "
            f"got {len(raw_data)}"
        )
    return [
        (float(x), float(y), float(z), float(intensity))
        for x, y, z, intensity in struct.iter_unpack("<ffff", raw_data)
    ]


class LidarSensorCapture:
    """Own one CARLA ray-cast LiDAR and synchronize callbacks by frame number."""

    def __init__(
        self,
        world: Any,
        carla_module: Any,
        host_vehicle: Any,
        config: LidarSensorConfig,
        *,
        client: Any | None = None,
        queue_size: int = 4,
    ) -> None:
        self.config = config
        self._carla_module = carla_module
        self._client = client
        self._queue: Queue[Any] = Queue(maxsize=queue_size)
        self._pending: dict[int, Any] = {}
        blueprint = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        blueprint.set_attribute("channels", str(config.channels))
        blueprint.set_attribute("range", str(config.lidar_range))
        blueprint.set_attribute("points_per_second", str(config.points_per_second))
        blueprint.set_attribute("rotation_frequency", str(config.rotation_frequency))
        blueprint.set_attribute("upper_fov", str(config.upper_fov_degrees))
        blueprint.set_attribute("lower_fov", str(config.lower_fov_degrees))
        if config.sensor_tick is not None:
            blueprint.set_attribute("sensor_tick", str(config.sensor_tick))

        transform = carla_module.Transform(
            carla_module.Location(x=config.x, y=config.y, z=config.z),
            carla_module.Rotation(
                roll=config.roll,
                pitch=config.pitch,
                yaw=config.yaw,
            ),
        )
        self.sensor = world.spawn_actor(blueprint, transform, attach_to=host_vehicle)
        self.sensor.listen(self._on_measurement)

    def _on_measurement(self, measurement: Any) -> None:
        try:
            self._queue.put_nowait(measurement)
        except Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(measurement)
            except Empty:
                return

    def get_for_frame(self, frame: int, timeout: float) -> Any:
        """Return the LiDAR measurement for ``frame``."""

        if frame in self._pending:
            return self._pending.pop(frame)

        while True:
            try:
                measurement = self._queue.get(timeout=timeout)
            except Empty as exc:
                raise TimeoutError(
                    f"Timed out waiting for LiDAR '{self.config.name}' frame {frame}"
                ) from exc
            measurement_frame = int(measurement.frame)
            if measurement_frame == frame:
                return measurement
            if measurement_frame > frame:
                self._pending[measurement_frame] = measurement
                raise TimeoutError(
                    f"LiDAR '{self.config.name}' skipped requested frame {frame}; "
                    f"received frame {measurement_frame}"
                )

    def destroy(self) -> None:
        self.sensor.stop()
        if self._client is not None:
            self._client.apply_batch(
                [self._carla_module.command.DestroyActor(self.sensor)]
            )
        else:
            self.sensor.destroy()


def lidar_topic(config: LidarSensorConfig) -> str:
    return f"sensor_view/{config.name}"
