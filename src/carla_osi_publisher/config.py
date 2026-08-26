"""Configuration objects for the publisher."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class GroundTruthConfig:
    """GroundTruth conversion policy."""

    ego: str | int | None = "hero"
    flip_y: bool = True
    include_static_objects: bool = True
    include_traffic_signs: bool = True
    include_traffic_lights: bool = True
    include_lane_network: bool = False
    map_reference: str | None = None
    model_reference: str | None = None
    country_code: int | None = None
    proj_string: str | None = None
    static_object_labels: tuple[str, ...] = (
        "Buildings",
        "Fences",
        "Other",
        "Poles",
        "RoadLines",
        "Roads",
        "Sidewalks",
        "Vegetation",
        "Walls",
        "Ground",
        "Bridge",
        "RailTrack",
        "GuardRail",
        "Static",
        "Water",
        "Terrain",
    )
    metadata: dict[str, str] = field(default_factory=dict)


@dataclass(slots=True)
class PublisherConfig:
    """Runtime configuration shared by future publishers."""

    host: str = "127.0.0.1"
    port: int = 2000
    timeout_seconds: float = 10.0
    sync: bool = False
    delta_seconds: float = 0.05
    ground_truth: GroundTruthConfig = field(default_factory=GroundTruthConfig)
