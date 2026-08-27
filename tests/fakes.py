from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Vector:
    x: float
    y: float
    z: float


@dataclass
class Rotation:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class Transform:
    location: Vector
    rotation: Rotation = field(default_factory=Rotation)


@dataclass
class Extent:
    x: float
    y: float
    z: float


@dataclass
class BoundingBox:
    location: Vector
    extent: Extent


@dataclass
class Attribute:
    id: str
    value: str


@dataclass
class EnvironmentObject:
    id: int
    name: str
    bounding_box: BoundingBox
    transform: Transform


class FakeActor:
    def __init__(
        self,
        actor_id: int,
        type_id: str,
        *,
        location: Vector | None = None,
        velocity: Vector | None = None,
        acceleration: Vector | None = None,
        angular_velocity: Vector | None = None,
        attributes: list[Attribute] | None = None,
        light_state: int = 0,
    ) -> None:
        self.id = actor_id
        self.type_id = type_id
        self.bounding_box = BoundingBox(Vector(0.0, 0.0, 0.0), Extent(2.0, 1.0, 0.75))
        self._transform = Transform(
            location or Vector(1.0, 2.0, 3.0),
            Rotation(yaw=90.0),
        )
        self._velocity = velocity or Vector(4.0, 5.0, 0.0)
        self._acceleration = acceleration or Vector(0.1, 0.2, 0.0)
        self._angular_velocity = angular_velocity or Vector(0.0, 0.0, 10.0)
        self.attributes = attributes or []
        self._light_state = light_state

    def get_transform(self) -> Transform:
        return self._transform

    def get_velocity(self) -> Vector:
        return self._velocity

    def get_acceleration(self) -> Vector:
        return self._acceleration

    def get_angular_velocity(self) -> Vector:
        return self._angular_velocity

    def get_location(self) -> Vector:
        return self._transform.location

    def get_light_state(self) -> int:
        return self._light_state


@dataclass
class SnapshotTimestamp:
    elapsed_seconds: float = 12.25


@dataclass
class FakeSnapshot:
    timestamp: SnapshotTimestamp = field(default_factory=SnapshotTimestamp)


class FakeWorld:
    def __init__(
        self,
        actors: list[FakeActor],
        environment_objects: dict[object, list[EnvironmentObject]] | None = None,
    ) -> None:
        self._actors = actors
        self._environment_objects = environment_objects or {}

    def get_snapshot(self) -> FakeSnapshot:
        return FakeSnapshot()

    def get_actors(self) -> list[FakeActor]:
        return self._actors

    def get_environment_objects(self, label: object) -> list[EnvironmentObject]:
        return self._environment_objects.get(label, [])
