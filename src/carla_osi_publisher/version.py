"""Project and protocol versions."""

from __future__ import annotations

from dataclasses import dataclass

__version__ = "0.1.0"


@dataclass(frozen=True, slots=True)
class OSIVersion:
    major: int
    minor: int
    patch: int

    def as_tuple(self) -> tuple[int, int, int]:
        return self.major, self.minor, self.patch

    def as_string(self) -> str:
        return f"{self.major}.{self.minor}.{self.patch}"


OSI_VERSION = OSIVersion(3, 8, 0)
