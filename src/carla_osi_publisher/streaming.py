"""CARLA to OSI StreamingUpdate conversion."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

from .config import GroundTruthConfig
from .groundtruth import GroundTruthBuilder, GroundTruthBuildResult
from .ids import IdMapper
from .osi import osi_message, set_interface_version


@dataclass(frozen=True, slots=True)
class StreamingUpdateBuildResult:
    """StreamingUpdate plus the source GroundTruth used for validation."""

    message: Any
    source_ground_truth: Any
    initial: bool
    actor_count: int
    moving_object_count: int
    stationary_object_count: int
    traffic_sign_count: int
    traffic_light_count: int


class StreamingUpdateBuilder:
    """Build an initial snapshot followed by partial OSI StreamingUpdates.

    The first message contains the complete static and dynamic state. Later
    messages contain moving objects and traffic lights, while static objects
    and traffic signs are retained by the receiver from the initial state.
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
        full_config = replace(self.config, include_lane_network=False)
        dynamic_config = replace(
            self.config,
            include_static_objects=False,
            include_traffic_signs=False,
            include_lane_network=False,
        )
        self._full_builder = GroundTruthBuilder(
            full_config,
            id_mapper=self.ids,
            carla_module=carla_module,
        )
        self._dynamic_builder = GroundTruthBuilder(
            dynamic_config,
            id_mapper=self.ids,
            carla_module=carla_module,
        )
        self._started = False
        self._previous_dynamic_ids: set[int] = set()

    @property
    def carla(self) -> Any | None:
        return self._full_builder.carla

    @carla.setter
    def carla(self, value: Any | None) -> None:
        self._full_builder.carla = value
        self._dynamic_builder.carla = value

    def build(self, world: Any, *, snapshot: Any | None = None) -> StreamingUpdateBuildResult:
        builder = self._full_builder if not self._started else self._dynamic_builder
        ground_truth_result = builder.build(world, snapshot=snapshot)
        message_class = osi_message("osi_streamingupdate_pb2", "StreamingUpdate")
        message = message_class()
        set_interface_version(message)
        message.timestamp.CopyFrom(ground_truth_result.message.timestamp)

        if not self._started:
            self._copy_messages(
                message.stationary_object_update,
                ground_truth_result.message.stationary_object,
            )
            self._copy_messages(
                message.traffic_sign_update,
                ground_truth_result.message.traffic_sign,
            )

        self._copy_messages(
            message.moving_object_update,
            ground_truth_result.message.moving_object,
        )
        self._copy_messages(
            message.traffic_light_update,
            ground_truth_result.message.traffic_light,
        )
        self._add_obsolete_ids(message, ground_truth_result)
        self._previous_dynamic_ids = self._dynamic_ids(ground_truth_result)
        initial = not self._started
        self._started = True

        return StreamingUpdateBuildResult(
            message=message,
            source_ground_truth=ground_truth_result.message,
            initial=initial,
            actor_count=ground_truth_result.actor_count,
            moving_object_count=len(message.moving_object_update),
            stationary_object_count=len(message.stationary_object_update),
            traffic_sign_count=len(message.traffic_sign_update),
            traffic_light_count=len(message.traffic_light_update),
        )

    def _add_obsolete_ids(self, message: Any, result: GroundTruthBuildResult) -> None:
        current_ids = self._dynamic_ids(result)
        for object_id in sorted(self._previous_dynamic_ids - current_ids):
            message.obsolete_id.add().value = object_id

    def _dynamic_ids(self, result: GroundTruthBuildResult) -> set[int]:
        return {
            object_id
            for object_id in (
                [item.id.value for item in result.message.moving_object if item.HasField("id")]
                + [item.id.value for item in result.message.traffic_light if item.HasField("id")]
            )
        }

    @staticmethod
    def _copy_messages(target: Any, source: Any) -> None:
        for source_message in source:
            target.add().CopyFrom(source_message)
