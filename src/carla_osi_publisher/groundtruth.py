"""CARLA world to OSI GroundTruth conversion."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

from .config import GroundTruthConfig
from .geometry import dimension3d, orientation3d, timestamp, vector3
from .ids import IdMapper
from .osi import osi_message, set_interface_version


@dataclass(frozen=True, slots=True)
class GroundTruthBuildResult:
    """GroundTruth plus source statistics useful for logging and tests."""

    message: Any
    actor_count: int
    moving_object_count: int
    stationary_object_count: int
    traffic_sign_count: int
    traffic_light_count: int


class GroundTruthBuilder:
    """Build OSI 3.8.0 GroundTruth from a CARLA world-like object.

    The builder uses CARLA's Python object shape through duck typing. This
    keeps the conversion logic testable without importing the CARLA package.
    """

    def __init__(
        self,
        config: GroundTruthConfig | None = None,
        *,
        id_mapper: IdMapper | None = None,
        carla_module: Any | None = None,
    ) -> None:
        self.config = config or GroundTruthConfig()
        self.ids = id_mapper or IdMapper()
        self.carla = carla_module

    def build(self, world: Any, *, snapshot: Any | None = None) -> GroundTruthBuildResult:
        """Build one GroundTruth message from the current CARLA world."""

        ground_truth_class = osi_message("osi_groundtruth_pb2", "GroundTruth")
        message = ground_truth_class()
        set_interface_version(message)

        snapshot = snapshot or self._snapshot(world)
        simulation_seconds = self._simulation_seconds(snapshot)
        timestamp(message.timestamp, simulation_seconds)

        if self.config.map_reference:
            message.map_reference = self.config.map_reference
        if self.config.model_reference:
            message.model_reference = self.config.model_reference
        if self.config.country_code is not None:
            message.country_code = self.config.country_code
        if self.config.proj_string:
            message.proj_string = self.config.proj_string

        actors = list(self._actors(world))
        host_id = self._resolve_host_id(actors)
        if host_id is not None:
            message.host_vehicle_id.value = self.ids.actor(host_id)

        moving_count = 0
        for actor in actors:
            type_id = str(getattr(actor, "type_id", ""))
            if type_id.startswith("vehicle."):
                self._add_vehicle(message, actor)
                moving_count += 1
            elif type_id.startswith("walker.pedestrian."):
                self._add_pedestrian(message, actor)
                moving_count += 1

        stationary_count = 0
        sign_count = 0
        light_count = 0
        if self.config.include_static_objects:
            stationary_count = self._add_static_objects(message, world)
        if self.config.include_traffic_signs:
            sign_count = self._add_traffic_signs(message, world)
        if self.config.include_traffic_lights:
            light_count = self._add_traffic_lights(message, actors)

        if self.config.include_lane_network:
            self._add_lane_network(message, world)

        return GroundTruthBuildResult(
            message=message,
            actor_count=len(actors),
            moving_object_count=moving_count,
            stationary_object_count=stationary_count,
            traffic_sign_count=sign_count,
            traffic_light_count=light_count,
        )

    @staticmethod
    def _snapshot(world: Any) -> Any:
        return world.get_snapshot()

    @staticmethod
    def _actors(world: Any) -> Iterable[Any]:
        actors = world.get_actors()
        return list(actors)

    @staticmethod
    def _simulation_seconds(snapshot: Any) -> float:
        timestamp_obj = getattr(snapshot, "timestamp", snapshot)
        return float(getattr(timestamp_obj, "elapsed_seconds", 0.0))

    def _resolve_host_id(self, actors: list[Any]) -> int | None:
        requested = self.config.ego
        if requested is None:
            return None
        for actor in actors:
            if not str(getattr(actor, "type_id", "")).startswith("vehicle."):
                continue
            actor_id = int(getattr(actor, "id"))
            if isinstance(requested, int) and actor_id == requested:
                return actor_id
            if isinstance(requested, str) and requested.isdigit() and actor_id == int(requested):
                return actor_id
            for attribute in self._attributes(actor):
                if self._attribute_id(attribute) == "role_name":
                    if self._attribute_value(attribute) == str(requested):
                        return actor_id
        return None

    def _add_vehicle(self, message: Any, actor: Any) -> None:
        moving = message.moving_object.add()
        moving.id.value = self.ids.actor(int(actor.id))
        moving.model_reference = str(getattr(actor, "type_id", ""))
        moving.type = self._enum("MovingObject", "TYPE_VEHICLE")
        self._fill_base_moving(moving.base, actor)
        self._fill_vehicle_classification(moving, actor)
        self._fill_vehicle_attributes(moving, actor)
        self._assign_nearest_lane(moving, actor)

    def _add_pedestrian(self, message: Any, actor: Any) -> None:
        moving = message.moving_object.add()
        moving.id.value = self.ids.actor(int(actor.id))
        moving.model_reference = str(getattr(actor, "type_id", ""))
        moving.type = self._enum("MovingObject", "TYPE_PEDESTRIAN")
        self._fill_base_moving(moving.base, actor)
        self._assign_nearest_lane(moving, actor)

    def _fill_base_moving(self, base: Any, actor: Any) -> None:
        transform = actor.get_transform()
        bbox = actor.bounding_box
        dimension3d(base.dimension, bbox)
        vector3(base.position, transform.location, flip_y=self.config.flip_y)
        orientation3d(base.orientation, transform.rotation, flip_yaw=self.config.flip_y)
        vector3(base.velocity, actor.get_velocity(), flip_y=self.config.flip_y)
        vector3(base.acceleration, actor.get_acceleration(), flip_y=self.config.flip_y)
        angular_velocity = actor.get_angular_velocity()
        orientation3d(
            base.orientation_rate,
            angular_velocity,
            flip_yaw=self.config.flip_y,
        )

    def _fill_vehicle_classification(self, moving: Any, actor: Any) -> None:
        classification = moving.vehicle_classification
        classification.has_trailer = False
        classification.type = self._vehicle_type(actor)
        light_state = getattr(actor, "get_light_state", lambda: None)()
        if light_state is not None:
            self._fill_light_state(classification.light_state, light_state)

    def _fill_vehicle_attributes(self, moving: Any, actor: Any) -> None:
        attributes = {self._attribute_id(a): self._attribute_value(a) for a in self._attributes(actor)}
        target = moving.vehicle_attributes
        self._set_number(target, "number_wheels", attributes.get("number_of_wheels"))
        self._set_number(target, "radius_wheel", attributes.get("wheel_radius"))
        self._set_number(target, "ground_clearance", attributes.get("ground_clearance"))
        for field_name, target_message in (
            ("bbcenter_to_front", target.bbcenter_to_front),
            ("bbcenter_to_rear", target.bbcenter_to_rear),
        ):
            for axis in ("x", "y", "z"):
                key = f"{field_name}_{axis}"
                if key in attributes:
                    setattr(target_message, axis, float(attributes[key]))
                    if self.config.flip_y and axis == "y":
                        setattr(target_message, axis, -float(attributes[key]))

    def _vehicle_type(self, actor: Any) -> int:
        values = {
            "OTHER": "TYPE_OTHER",
            "SMALL_CAR": "TYPE_SMALL_CAR",
            "COMPACT_CAR": "TYPE_COMPACT_CAR",
            "MEDIUM_CAR": "TYPE_MEDIUM_CAR",
            "LUXURY_CAR": "TYPE_LUXURY_CAR",
            "DELIVERY_VAN": "TYPE_DELIVERY_VAN",
            "HEAVY_TRUCK": "TYPE_HEAVY_TRUCK",
            "SEMITRACTOR": "TYPE_SEMITRACTOR",
            "SEMITRAILER": "TYPE_SEMITRAILER",
            "TRAILER": "TYPE_TRAILER",
            "MOTORBIKE": "TYPE_MOTORBIKE",
            "BICYCLE": "TYPE_BICYCLE",
            "BUS": "TYPE_BUS",
            "TRAM": "TYPE_TRAM",
            "TRAIN": "TYPE_TRAIN",
            "WHEELCHAIR": "TYPE_WHEELCHAIR",
        }
        value = ""
        for attribute in self._attributes(actor):
            if self._attribute_id(attribute) in {"osi_vehicle_type", "object_type"}:
                value = self._attribute_value(attribute)
                break
        for key, enum_name in values.items():
            if key in value.upper():
                return self._enum("VehicleClassification", enum_name)
        return self._enum("VehicleClassification", "TYPE_UNKNOWN")

    def _fill_light_state(self, target: Any, light_state: Any) -> None:
        flags = self._light_flags(light_state)
        generic = "MovingObject.VehicleClassification.LightState.GenericLightState"
        brake = "MovingObject.VehicleClassification.LightState.BrakeLightState"
        target.head_light = self._enum(
            generic,
            "GENERIC_LIGHT_STATE_ON" if flags & 8 else "GENERIC_LIGHT_STATE_OFF",
        )
        target.high_beam = self._enum(
            generic,
            "GENERIC_LIGHT_STATE_ON" if flags & 16 else "GENERIC_LIGHT_STATE_OFF",
        )
        target.reversing_light = self._enum(
            generic,
            "GENERIC_LIGHT_STATE_ON" if flags & 32 else "GENERIC_LIGHT_STATE_OFF",
        )
        target.brake_light_state = self._enum(
            brake,
            "BRAKE_LIGHT_STATE_NORMAL" if flags & 4 else "BRAKE_LIGHT_STATE_OFF",
        )
        if flags & 64 and flags & 128:
            indicator = "INDICATOR_STATE_WARNING"
        elif flags & 64:
            indicator = "INDICATOR_STATE_LEFT"
        elif flags & 128:
            indicator = "INDICATOR_STATE_RIGHT"
        else:
            indicator = "INDICATOR_STATE_OFF"
        target.indicator_state = self._enum(
            "MovingObject.VehicleClassification.LightState.IndicatorState",
            indicator,
        )

    @staticmethod
    def _light_flags(light_state: Any) -> int:
        try:
            return int(light_state)
        except (TypeError, ValueError):
            return int(getattr(light_state, "value", 0))

    def _add_static_objects(self, message: Any, world: Any) -> int:
        count = 0
        label_enum = getattr(self.carla, "CityObjectLabel", None) if self.carla else None
        if label_enum is None:
            return count
        for label_name in self.config.static_object_labels:
            label = getattr(label_enum, label_name, None)
            if label is None:
                continue
            for obj in world.get_environment_objects(label):
                stationary = message.stationary_object.add()
                stationary.id.value = self.ids.environment(int(obj.id))
                stationary.model_reference = str(getattr(obj, "name", ""))
                dimension3d(stationary.base.dimension, obj.bounding_box)
                vector3(
                    stationary.base.position,
                    obj.transform.location,
                    flip_y=self.config.flip_y,
                )
                orientation3d(
                    stationary.base.orientation,
                    obj.transform.rotation,
                    flip_yaw=self.config.flip_y,
                )
                stationary.classification.type = self._stationary_type(label_name)
                count += 1
        return count

    def _add_traffic_signs(self, message: Any, world: Any) -> int:
        if not hasattr(world, "get_environment_objects") or self.carla is None:
            return 0
        label = getattr(getattr(self.carla, "CityObjectLabel", None), "TrafficSigns", None)
        if label is None:
            return 0
        count = 0
        for obj in world.get_environment_objects(label):
            sign = message.traffic_sign.add()
            sign.id.value = self.ids.environment(int(obj.id))
            self._fill_stationary_base(sign.main_sign.base, obj)
            name = str(getattr(obj, "name", ""))
            sign.main_sign.classification.variability = self._enum(
                "TrafficSign.Variability",
                "VARIABILITY_FIXED",
            )
            sign.main_sign.classification.direction_scope = self._enum(
                "TrafficSign.MainSign.Classification.DirectionScope",
                "DIRECTION_SCOPE_NO_DIRECTION",
            )
            self._fill_sign_type(sign.main_sign.classification, name)
            count += 1
        return count

    def _add_traffic_lights(self, message: Any, actors: list[Any]) -> int:
        count = 0
        for actor in actors:
            if str(getattr(actor, "type_id", "")) != "traffic.traffic_light":
                continue
            light = message.traffic_light.add()
            light.id.value = self.ids.traffic_light(int(actor.id))
            self._fill_actor_stationary_base(light.base, actor)
            state = str(getattr(actor, "get_state", lambda: "")()).upper()
            colors = {
                "RED": "COLOR_RED",
                "YELLOW": "COLOR_YELLOW",
                "GREEN": "COLOR_GREEN",
                "OFF": "COLOR_OTHER",
            }
            modes = {
                "RED": "MODE_CONSTANT",
                "YELLOW": "MODE_CONSTANT",
                "GREEN": "MODE_CONSTANT",
                "OFF": "MODE_OFF",
            }
            light.classification.color = self._enum(
                "TrafficLight.Classification.Color",
                colors.get(state, "COLOR_OTHER"),
            )
            light.classification.mode = self._enum(
                "TrafficLight.Classification.Mode",
                modes.get(state, "MODE_OTHER"),
            )
            light.classification.icon = self._enum(
                "TrafficLight.Classification.Icon",
                "ICON_NONE",
            )
            count += 1
        return count

    def _add_lane_network(self, message: Any, world: Any) -> None:
        """Reserved for the next GroundTruth increment."""

        del message, world

    def _assign_nearest_lane(self, moving: Any, actor: Any) -> None:
        if not self.config.include_lane_network:
            return
        world = getattr(actor, "get_world", lambda: None)()
        carla_map = world.get_map() if world is not None and hasattr(world, "get_map") else None
        if carla_map is None or not hasattr(carla_map, "get_waypoint"):
            return
        waypoint = carla_map.get_waypoint(actor.get_location())
        if waypoint is None:
            return
        road_id = int(getattr(waypoint, "road_id", 0))
        lane_id = int(getattr(waypoint, "lane_id", 0))
        section_id = int(getattr(waypoint, "section_id", 0))
        raw = ((road_id & 0xFFFFFFFF) << 24) ^ ((lane_id & 0xFFFF) << 8) ^ (section_id & 0xFF)
        moving.assigned_lane_id.add().value = self.ids._encode(4, raw)

    def _fill_stationary_base(self, base: Any, obj: Any) -> None:
        dimension3d(base.dimension, obj.bounding_box)
        vector3(base.position, obj.transform.location, flip_y=self.config.flip_y)
        orientation3d(base.orientation, obj.transform.rotation, flip_yaw=self.config.flip_y)

    def _fill_actor_stationary_base(self, base: Any, actor: Any) -> None:
        dimension3d(base.dimension, actor.bounding_box)
        transform = actor.get_transform()
        vector3(base.position, transform.location, flip_y=self.config.flip_y)
        orientation3d(base.orientation, transform.rotation, flip_yaw=self.config.flip_y)

    def _fill_sign_type(self, classification: Any, name: str) -> None:
        prefix = "TrafficSign.MainSign.Classification.Type"
        if name.startswith("traffic.speed_limit."):
            classification.type = self._enum(prefix, "TYPE_SPEED_LIMIT_BEGIN")
            value = name.rsplit(".", 1)[-1]
            classification.value.value = float(value) if value.isdigit() else 0.0
            classification.value.value_unit = self._enum(
                "TrafficSignValue.Unit",
                "UNIT_KILOMETER_PER_HOUR",
            )
        elif name == "traffic.stop":
            classification.type = self._enum(prefix, "TYPE_STOP")
        elif name == "traffic.yield":
            classification.type = self._enum(prefix, "TYPE_GIVE_WAY")
        else:
            classification.type = self._enum(prefix, "TYPE_OTHER")

    def _stationary_type(self, label_name: str) -> int:
        enum_name = {
            "Buildings": "TYPE_BUILDING",
            "Fences": "TYPE_BARRIER",
            "GuardRail": "TYPE_BARRIER",
            "Poles": "TYPE_POLE",
            "Vegetation": "TYPE_VEGETATION",
            "Walls": "TYPE_WALL",
            "Bridge": "TYPE_BRIDGE",
        }.get(label_name, "TYPE_OTHER")
        return self._enum("StationaryObject.Classification.Type", enum_name)

    @staticmethod
    def _attributes(actor: Any) -> Iterable[Any]:
        attributes = getattr(actor, "attributes", []) or []
        if isinstance(attributes, Mapping):
            return attributes.items()
        return attributes

    @staticmethod
    def _attribute_id(attribute: Any) -> str:
        if isinstance(attribute, tuple) and len(attribute) == 2:
            return str(attribute[0])
        return str(getattr(attribute, "id", ""))

    @staticmethod
    def _attribute_value(attribute: Any) -> str:
        if isinstance(attribute, tuple) and len(attribute) == 2:
            return str(attribute[1])
        return str(getattr(attribute, "value", ""))

    @staticmethod
    def _set_number(target: Any, field_name: str, value: str | None) -> None:
        if value is None or value == "":
            return
        try:
            setattr(target, field_name, float(value))
        except (TypeError, ValueError):
            return

    @staticmethod
    def _enum(path: str, name: str) -> int:
        """Resolve a nested OSI enum value from its protobuf descriptor path."""

        parts = path.split(".")
        module_name = {
            "MovingObject": "osi_object_pb2",
            "StationaryObject": "osi_object_pb2",
            "TrafficSign": "osi_trafficsign_pb2",
            "TrafficSignValue": "osi_trafficsign_pb2",
            "TrafficLight": "osi_trafficlight_pb2",
            "VehicleClassification": "osi_object_pb2",
        }.get(parts[0], "osi_object_pb2")
        if parts[0] == "VehicleClassification":
            parts.insert(0, "MovingObject")

        descriptor = osi_message(module_name, parts[0]).DESCRIPTOR
        for part in parts[1:]:
            nested_message = descriptor.nested_types_by_name.get(part)
            if nested_message is not None:
                descriptor = nested_message
                continue
            enum_descriptor = descriptor.enum_types_by_name.get(part)
            if enum_descriptor is not None:
                value = enum_descriptor.values_by_name.get(name)
                if value is None:
                    raise KeyError(f"Unknown enum value {name!r} in {path!r}")
                return value.number
            raise KeyError(f"Unknown protobuf descriptor component {part!r} in {path!r}")

        for enum_descriptor in descriptor.enum_types:
            value = enum_descriptor.values_by_name.get(name)
            if value is not None:
                return value.number
        raise KeyError(f"Unknown enum value {name!r} in {path!r}")
