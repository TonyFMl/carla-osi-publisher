"""Deterministic namespaced OSI identifiers."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field


@dataclass(slots=True)
class IdMapper:
    """Encode CARLA identifiers into globally separated OSI identifier spaces.

    CARLA actor IDs are usually small integers, but environment object IDs can
    use the full unsigned 64-bit range. OSI identifiers reserve the top eight
    bits for the namespace, so oversized source IDs are deterministically
    folded into the remaining 56 bits. Collisions are resolved within this
    mapper instance.
    """

    actor_namespace: int = 1
    environment_namespace: int = 2
    traffic_light_namespace: int = 3
    sensor_namespace: int = 5
    namespace_shift: int = 56
    _payload_by_source: dict[tuple[int, int], int] = field(
        default_factory=dict,
        init=False,
        repr=False,
        compare=False,
    )
    _source_by_payload: dict[tuple[int, int], int] = field(
        default_factory=dict,
        init=False,
        repr=False,
        compare=False,
    )

    def _encode(self, namespace: int, value: int) -> int:
        if value < 0:
            raise ValueError("CARLA identifiers must be non-negative")
        if namespace < 0 or namespace >= 1 << 8:
            raise ValueError("namespace must fit in 8 bits")

        source_key = (namespace, value)
        payload = self._payload_by_source.get(source_key)
        if payload is None:
            payload_limit = 1 << self.namespace_shift
            if value < payload_limit:
                candidate = value
            else:
                digest = hashlib.blake2b(
                    f"{namespace}:{value}".encode("ascii"),
                    digest_size=8,
                ).digest()
                candidate = int.from_bytes(digest, "big") & (payload_limit - 1)

            payload = candidate
            while True:
                existing = self._source_by_payload.get((namespace, payload))
                if existing is None or existing == value:
                    break
                payload = (payload + 1) & (payload_limit - 1)
            self._payload_by_source[source_key] = payload
            self._source_by_payload[(namespace, payload)] = value

        return (namespace << self.namespace_shift) | payload

    def actor(self, actor_id: int) -> int:
        return self._encode(self.actor_namespace, int(actor_id))

    def environment(self, object_id: int) -> int:
        return self._encode(self.environment_namespace, int(object_id))

    def traffic_light(self, actor_id: int, bulb_index: int = 0) -> int:
        raw = (int(actor_id) << 8) | (int(bulb_index) & 0xFF)
        return self._encode(self.traffic_light_namespace, raw)

    def sensor(self, actor_id: int) -> int:
        return self._encode(self.sensor_namespace, int(actor_id))
