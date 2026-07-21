"""Python SDK for realtime IMU data from the Auli.tech Cato wearable."""

from .device import Cato
from .exceptions import (
    CatoError,
    CommandError,
    CommandTimeoutError,
    DeviceDisconnectedError,
    DeviceNotFoundError,
)
from .fusion import MadgwickFilter
from .models import GestureResult, ImuSample, PracticeEvent, StatusEvent

__version__ = "0.1.0"

__all__ = [
    "Cato",
    "CatoError",
    "CommandError",
    "CommandTimeoutError",
    "DeviceDisconnectedError",
    "DeviceNotFoundError",
    "MadgwickFilter",
    "GestureResult",
    "ImuSample",
    "PracticeEvent",
    "StatusEvent",
    "__version__",
]
