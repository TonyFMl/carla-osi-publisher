"""Errors raised by the publisher."""


class CarlaUnavailableError(RuntimeError):
    """Raised when a CARLA operation is requested without the CARLA package."""


class CarlaConnectionError(RuntimeError):
    """Raised when the CARLA client cannot connect to a server."""


class UnsupportedCarlaDataError(ValueError):
    """Raised when an input object cannot be converted safely."""
