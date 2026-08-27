from __future__ import annotations

from types import SimpleNamespace

from carla_osi_publisher.sensorview import (
    CameraSensorConfig,
    CameraSensorViewBuilder,
    LidarSensorConfig,
    LidarSensorViewBuilder,
)


def test_camera_sensor_view_converts_bgra_and_populates_configuration() -> None:
    image = SimpleNamespace(
        width=2,
        height=1,
        timestamp=1.25,
        raw_data=bytes([1, 2, 3, 4, 10, 20, 30, 40]),
    )
    sensor = SimpleNamespace(id=5)
    host_vehicle = SimpleNamespace(id=7)
    config = CameraSensorConfig(
        name="camera_front",
        width=2,
        height=1,
        fov_degrees=90.0,
        y=1.0,
        z=2.4,
        yaw=90.0,
    )

    message = CameraSensorViewBuilder(config).build(
        image,
        sensor,
        host_vehicle,
    )

    camera_view = message.camera_sensor_view[0]
    camera_config = camera_view.view_configuration
    assert (message.version.version_major, message.version.version_minor) == (3, 8)
    assert (message.timestamp.seconds, message.timestamp.nanos) == (1, 250_000_000)
    assert message.sensor_id.value == (5 << 56) | 5
    assert message.host_vehicle_id.value == (1 << 56) | 7
    assert camera_view.image_data == bytes([3, 2, 1, 30, 20, 10])
    assert (camera_config.number_of_pixels_horizontal, camera_config.number_of_pixels_vertical) == (2, 1)
    assert list(camera_config.channel_format) == [camera_config.CHANNEL_FORMAT_RGB_U8_LIN]
    assert camera_config.samples_per_pixel == 3
    assert camera_config.mounting_position.position.y == -1.0
    assert round(camera_config.mounting_position.orientation.yaw, 6) == -1.570796


def test_camera_sensor_view_vertical_fov_uses_image_aspect_ratio() -> None:
    wide = CameraSensorConfig(name="wide", width=400, height=300)
    square = CameraSensorConfig(name="square", width=400, height=400)

    assert wide.vertical_fov_radians < square.vertical_fov_radians


def test_lidar_sensor_view_converts_points_to_reflections_and_directions() -> None:
    import struct

    measurement = SimpleNamespace(
        timestamp=2.5,
        frame=12,
        raw_data=struct.pack(
            "<8f",
            10.0,
            2.0,
            0.0,
            0.75,
            0.0,
            -4.0,
            3.0,
            0.25,
        ),
    )
    sensor = SimpleNamespace(id=9)
    host_vehicle = SimpleNamespace(id=7)
    config = LidarSensorConfig(
        name="lidar_0",
        channels=4,
        points_per_second=400,
        rotation_frequency=10.0,
        upper_fov_degrees=15.0,
        lower_fov_degrees=-25.0,
    )

    message = LidarSensorViewBuilder(config).build(
        measurement,
        sensor,
        host_vehicle,
    )

    lidar_view = message.lidar_sensor_view[0]
    lidar_config = lidar_view.view_configuration
    assert (message.version.version_major, message.version.version_minor) == (3, 8)
    assert (message.timestamp.seconds, message.timestamp.nanos) == (2, 500_000_000)
    assert message.sensor_id.value == (5 << 56) | 9
    assert message.host_vehicle_id.value == (1 << 56) | 7
    assert lidar_config.num_of_pixels == 2
    assert len(lidar_config.directions) == 2
    assert len(lidar_view.reflection) == 2
    assert lidar_config.number_of_rays_vertical == 4
    assert lidar_config.number_of_rays_horizontal == 10
    assert lidar_config.directions[0].y < 0
    assert lidar_view.reflection[0].signal_strength == 0.75
    assert lidar_view.reflection[0].time_of_flight > 0
