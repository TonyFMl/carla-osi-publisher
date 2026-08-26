"""CARLA 0.9.16 connection and stepping adapter."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

from .config import PublisherConfig
from .errors import CarlaConnectionError
from .osi import ensure_carla


class CarlaClient:
    """Small adapter around the CARLA Python client."""

    def __init__(self, config: PublisherConfig) -> None:
        self.config = config
        self.client: Any | None = None
        self.world: Any | None = None
        self.carla: Any | None = None
        self._original_sync_settings: tuple[bool, float | None] | None = None

    def connect(self) -> Any:
        self.carla = ensure_carla()
        try:
            self.client = self.carla.Client(self.config.host, self.config.port)
            self.client.set_timeout(self.config.timeout_seconds)
            self.world = self.client.get_world()
        except Exception as exc:
            raise CarlaConnectionError(
                f"Could not connect to CARLA at {self.config.host}:{self.config.port}: {exc}"
            ) from exc
        if self.config.sync:
            self.enable_synchronous_mode()
        return self.world

    def enable_synchronous_mode(self) -> None:
        if self.world is None:
            raise CarlaConnectionError("Connect to CARLA before changing world settings")
        settings = self.world.get_settings()
        if self._original_sync_settings is None:
            self._original_sync_settings = (
                bool(settings.synchronous_mode),
                settings.fixed_delta_seconds,
            )
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.config.delta_seconds
        self.world.apply_settings(settings)

    def tick(self) -> Any:
        if self.world is None:
            raise CarlaConnectionError("Connect to CARLA before ticking")
        if self.config.sync:
            self.world.tick()
        return self.world.get_snapshot()

    def wait_for_tick(self) -> Any:
        if self.world is None:
            raise CarlaConnectionError("Connect to CARLA before waiting for a tick")
        return self.world.wait_for_tick(self.config.timeout_seconds)

    @contextmanager
    def connected(self) -> Iterator[Any]:
        world = self.connect()
        try:
            yield world
        finally:
            self.close()

    def close(self) -> None:
        if self.world is not None and self._original_sync_settings is not None:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode, settings.fixed_delta_seconds = self._original_sync_settings
                self.world.apply_settings(settings)
            except Exception:
                pass
        self.client = None
        self.world = None
        self.carla = None
        self._original_sync_settings = None
