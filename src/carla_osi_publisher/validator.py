"""OSI GroundTruth validation for the MVP."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .version import OSI_VERSION


@dataclass(slots=True)
class ValidationReport:
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return not self.errors


class GroundTruthValidator:
    """Validate the fields that the GroundTruth MVP promises to populate."""

    def validate(self, message: Any, *, require_host_vehicle: bool = False) -> ValidationReport:
        report = ValidationReport()
        if not message.HasField("version"):
            report.errors.append("GroundTruth.version is not set")
        else:
            actual = (
                message.version.version_major,
                message.version.version_minor,
                message.version.version_patch,
            )
            if actual != OSI_VERSION.as_tuple():
                report.errors.append(
                    f"GroundTruth.version is {actual}, expected {OSI_VERSION.as_tuple()}"
                )
        if not message.HasField("timestamp"):
            report.errors.append("GroundTruth.timestamp is not set")
        if require_host_vehicle and not message.HasField("host_vehicle_id"):
            report.errors.append("GroundTruth.host_vehicle_id is not set")

        object_ids: set[int] = set()
        moving_ids: set[int] = set()
        for index, moving in enumerate(message.moving_object):
            self._validate_object(report, moving, f"moving_object[{index}]", object_ids)
            if moving.HasField("id"):
                moving_ids.add(moving.id.value)
            if moving.type == 0:
                report.errors.append(f"moving_object[{index}].type is unknown")
            if moving.type == 2 and moving.vehicle_classification.type == 0:
                report.warnings.append(
                    f"moving_object[{index}].vehicle_classification.type is unknown"
                )

        for index, stationary in enumerate(message.stationary_object):
            self._validate_object(report, stationary, f"stationary_object[{index}]", object_ids)

        for index, sign in enumerate(message.traffic_sign):
            path = f"traffic_sign[{index}]"
            self._validate_identifier(report, sign, path, object_ids)
            if not sign.HasField("main_sign"):
                report.errors.append(f"{path}.main_sign is not set")
            elif not sign.main_sign.HasField("base"):
                report.errors.append(f"{path}.main_sign.base is not set")
            else:
                self._validate_base(report, sign.main_sign.base, f"{path}.main_sign")

        for index, light in enumerate(message.traffic_light):
            path = f"traffic_light[{index}]"
            self._validate_identifier(report, light, path, object_ids)
            if not light.HasField("base"):
                report.errors.append(f"{path}.base is not set")
            else:
                self._validate_base(report, light.base, path)

        if message.HasField("host_vehicle_id"):
            host_id = message.host_vehicle_id.value
            if host_id not in moving_ids:
                report.errors.append("host_vehicle_id does not refer to a moving_object")
        return report

    @staticmethod
    def _validate_object(report: ValidationReport, obj: Any, path: str, ids: set[int]) -> None:
        GroundTruthValidator._validate_identifier(report, obj, path, ids)
        if not obj.HasField("base"):
            report.errors.append(f"{path}.base is not set")
            return
        GroundTruthValidator._validate_base(report, obj.base, path)

    @staticmethod
    def _validate_identifier(report: ValidationReport, obj: Any, path: str, ids: set[int]) -> None:
        if not obj.HasField("id"):
            report.errors.append(f"{path}.id is not set")
        elif obj.id.value in ids:
            report.errors.append(f"{path}.id is duplicated")
        else:
            ids.add(obj.id.value)

    @staticmethod
    def _validate_base(report: ValidationReport, base: Any, path: str) -> None:
        for field_name in ("dimension", "position", "orientation"):
            if not base.HasField(field_name):
                report.errors.append(f"{path}.base.{field_name} is not set")
