"""Data types returned by the SDK.

Frame convention
----------------
``acc`` and ``gyro`` are given in the *reference frame* used by Auli's
imu_visualizer demo (Y-up, device axes remapped to x/y/z, gyro y/z negated).
This is the frame the sensor-fusion constants were tuned in, so orientation
output matches the official demo. The untouched on-wire values are always
available in ``ImuSample.raw``.

At rest, ``acc`` reads approximately ``(0, -9.81, 0)`` (gravity along -Y).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class ImuSample:
    """One IMU sample from the sensor stream (input report 7)."""

    device_ts: int
    """Raw uint32 timestamp counter from the device."""

    host_ts: float
    """``time.monotonic()`` on the host when the report was read."""

    acc: Tuple[float, float, float]
    """Accelerometer in m/s², reference frame (see module docstring)."""

    gyro: Tuple[float, float, float]
    """Gyroscope in deg/s, reference frame (y and z negated vs. wire)."""

    raw: Tuple[float, ...]
    """Untouched wire values: (ts, acc_y, acc_z, acc_x, gyro_y, gyro_z, gyro_x)."""

    quaternion: Optional[Tuple[float, float, float, float]] = None
    """Fused orientation (w, x, y, z); None if fusion is disabled."""

    linear_acc: Optional[Tuple[float, float, float]] = None
    """Gravity-removed acceleration in the world frame, smoothed; None if fusion is disabled."""

    def euler(self) -> Tuple[float, float, float]:
        """Fused orientation as (roll, pitch, yaw) in degrees.

        Decomposed in intrinsic Y-X-Z order (three.js "YXZ"): yaw about the
        world Y (up) axis, pitch about X, roll about Z. Raises ``ValueError``
        if fusion is disabled.
        """
        if self.quaternion is None:
            raise ValueError("No quaternion on this sample (fusion disabled)")
        w, x, y, z = self.quaternion
        pitch = math.asin(max(-1.0, min(1.0, -2.0 * (y * z - w * x))))
        yaw = math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y))
        roll = math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z))
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


@dataclass(frozen=True)
class StatusEvent:
    """Device status (input report 9). The SDK answers the required heartbeat
    automatically before this event is dispatched."""

    gesture: int
    action: int
    con_idx: int
    hb_lost: bool
    deep_sleep: bool
    unsaved_changes: bool
    pointer_sleep: bool
    scrolling: bool
    practice: bool
    hold_til_idle: bool
    dwell_click: bool


@dataclass(frozen=True)
class GestureResult:
    """One gesture inference entry inside a practice report."""

    index: int
    confidence: int
    threshold_crossed: bool


@dataclass(frozen=True)
class PracticeEvent:
    """Gesture-inference report (input report 8)."""

    device_ts: int
    latched: int
    gestures: Tuple[GestureResult, ...]
