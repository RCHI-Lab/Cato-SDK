import math

from cato.fusion import GRAVITY, MadgwickFilter, _conjugate, _rotate

REST_ACC = (0.0, -GRAVITY, 0.0)  # gravity along -Y in the reference frame


def gravity_alignment(q):
    """Dot product between predicted and ideal body-frame gravity (1.0 = aligned)."""
    gx, gy, gz = _rotate(_conjugate(q), (0.0, -1.0, 0.0))
    return -gy  # ideal body gravity is (0, -1, 0)


class TestConvergence:
    def test_rest_is_stable_at_identity(self):
        f = MadgwickFilter()
        for _ in range(100):
            q = f.update((0.0, 0.0, 0.0), REST_ACC, 0.01)
        assert abs(q[0]) > 0.999  # still ~identity

    def test_converges_from_tilted_start(self):
        # start rolled 30 degrees about Z
        half = math.radians(30) / 2
        f = MadgwickFilter(initial=(math.cos(half), 0.0, 0.0, math.sin(half)))
        assert gravity_alignment(f.q) < 0.9
        for _ in range(2000):
            f.update((0.0, 0.0, 0.0), REST_ACC, 0.01)
        assert gravity_alignment(f.q) > 0.999


class TestGyroIntegration:
    def test_yaw_rate_integrates_to_expected_angle(self):
        # unit gyro scale so 90 deg/s for 1 s = 90 degrees about Y
        f = MadgwickFilter(gyro_scale=(1.0, 1.0, 1.0))
        for _ in range(100):
            q = f.update((0.0, 90.0, 0.0), REST_ACC, 0.01)
        angle = math.degrees(2 * math.acos(min(1.0, abs(q[0]))))
        assert abs(angle - 90.0) < 2.0
        # rotation axis is Y
        assert abs(q[2]) > 0.99 * math.sin(math.radians(45))
        assert abs(q[1]) < 0.05 and abs(q[3]) < 0.05


class TestLinearAcceleration:
    def test_zero_at_rest(self):
        f = MadgwickFilter()
        lin = f.linear_acceleration(REST_ACC)
        assert all(abs(v) < 1e-9 for v in lin)

    def test_smoothing_approaches_step_input(self):
        f = MadgwickFilter()
        acc = (1.0, -GRAVITY, 0.0)  # 1 m/s² sideways on top of gravity
        for _ in range(100):
            lin = f.linear_acceleration(acc)
        assert abs(lin[0] - 1.0) < 0.01
        assert abs(lin[1]) < 0.01 and abs(lin[2]) < 0.01


class TestReset:
    def test_reset_restores_initial_state(self):
        f = MadgwickFilter()
        for _ in range(50):
            f.update((100.0, 50.0, 25.0), REST_ACC, 0.01)
        assert abs(f.q[0]) < 0.999
        f.reset()
        assert f.q == (1.0, 0.0, 0.0, 0.0)
