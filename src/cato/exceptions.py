"""Exception types raised by the Cato SDK."""


class CatoError(Exception):
    """Base class for all Cato SDK errors."""


class DeviceNotFoundError(CatoError):
    """No Cato device was found on the system."""


class DeviceDisconnectedError(CatoError):
    """The device disconnected (or the HID handle died) mid-operation."""


class CommandError(CatoError):
    """The device rejected a command (NACK / empty / malformed response)."""


class CommandTimeoutError(CommandError):
    """The device did not answer a command within the timeout."""
