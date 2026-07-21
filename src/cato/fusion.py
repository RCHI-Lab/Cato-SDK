"""Madgwick sensor fusion, ported from imu_visualizer's ``Visualizer.jsx``.

IMU-only (accelerometer + gyroscope) Madgwick filter with the gravity
reference along the world -Y axis, matching the Y-up frame of the reference
demo. Quaternions are (w, x, y, z).
"""

from __future__ import annotations

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

DEG2RAD = math.pi / 180.0
GRAVITY = 9.81

DEFAULT_BETA = 0.6
# Per-axis gyro gain slightly above 1.0 to reduce gyro lag (from the demo).
DEFAULT_GYRO_SCALE: Vec3 = (1.15, 1.14, 1.17)


def _rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector v by unit quaternion q."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + q_vec × t
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _conjugate(q: Quat) -> Quat:
    w, x, y, z = q
    return (w, -x, -y, -z)


class MadgwickFilter:
    """Stateful orientation filter.

    Feed it reference-frame samples (``ImuSample.acc`` / ``ImuSample.gyro``)
    via :meth:`update`; read the fused quaternion back. ``linear_acceleration``
    additionally removes gravity and returns a smoothed world-frame vector.
    """

    def __init__(
        self,
        beta: float = DEFAULT_BETA,
        gyro_scale: Vec3 = DEFAULT_GYRO_SCALE,
        smoothing: float = 0.1,
        initial: Quat = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        self.beta = beta
        self.gyro_scale = gyro_scale
        self.smoothing = smoothing
        self._initial = initial
        self.q: Quat = initial
        self._smoothed_acc = [0.0, 0.0, 0.0]

    def reset(self) -> None:
        """Return to the initial orientation and clear smoothing state."""
        self.q = self._initial
        self._smoothed_acc = [0.0, 0.0, 0.0]

    def update(self, gyro_dps: Vec3, acc: Vec3, dt: float) -> Quat:
        """Advance the filter by one sample.

        ``gyro_dps`` in deg/s and ``acc`` in m/s², both in the reference
        frame; ``dt`` in seconds. Returns the new quaternion (w, x, y, z).
        """
        gx = gyro_dps[0] * DEG2RAD * self.gyro_scale[0]
        gy = gyro_dps[1] * DEG2RAD * self.gyro_scale[1]
        gz = gyro_dps[2] * DEG2RAD * self.gyro_scale[2]
        ax, ay, az = acc

        q0, q1, q2, q3 = self.q

        norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        ax /= norm
        ay /= norm
        az /= norm

        # Gradient for gravity reference along the -Y world axis
        q1q3sq = q1 * q1 + q3 * q3
        s0 = 4 * q0 * q1q3sq + 2 * q3 * ax - 2 * q1 * az
        s1 = 4 * q1 * q1q3sq + 2 * q2 * ax - 4 * q1 * ay - 2 * q0 * az
        s2 = 4 * q2 * q1q3sq + 2 * q1 * ax + 2 * q3 * az
        s3 = 4 * q3 * q1q3sq + 2 * q0 * ax - 4 * q3 * ay + 2 * q2 * az
        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) or 1.0
        s0 /= norm
        s1 /= norm
        s2 /= norm
        s3 /= norm

        n0 = q0 + (0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0) * dt
        n1 = q1 + (0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * s1) * dt
        n2 = q2 + (0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * s2) * dt
        n3 = q3 + (0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * s3) * dt
        norm = math.sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3) or 1.0
        self.q = (n0 / norm, n1 / norm, n2 / norm, n3 / norm)
        return self.q

    def linear_acceleration(self, acc: Vec3) -> Vec3:
        """Gravity-removed acceleration in the world frame, exponentially smoothed."""
        grav_body = _rotate(_conjugate(self.q), (0.0, -GRAVITY, 0.0))
        body = (acc[0] - grav_body[0], acc[1] - grav_body[1], acc[2] - grav_body[2])
        world = _rotate(self.q, body)
        a = self.smoothing
        s = self._smoothed_acc
        s[0] += (world[0] - s[0]) * a
        s[1] += (world[1] - s[1]) * a
        s[2] += (world[2] - s[2]) * a
        return (s[0], s[1], s[2])
