import numpy as np
import pytest

from kalman_core import (ConstantAccelerationModel2D, KalmanFilter,
                         PositionMeasurement, PositionVelocityMeasurement)

DT = 0.1


def _is_symmetric_psd(matrix: np.ndarray) -> bool:
    return (np.allclose(matrix, matrix.T)
            and np.linalg.eigvalsh(0.5 * (matrix + matrix.T)).min() >= -1e-9)


def _make_filter() -> KalmanFilter:
    return KalmanFilter(ConstantAccelerationModel2D(sigma_acc=0.2),
                        np.array([5.0, -5.0, 0.0, 0.0]),
                        np.diag([25.0, 25.0, 4.0, 4.0]))


def test_model_matrix_shapes():
    model = ConstantAccelerationModel2D(sigma_acc=0.2)
    assert model.state_dim == 4
    assert model.transition(DT).shape == (4, 4)
    assert model.control(DT).shape == (4, 2)
    assert model.process_noise(DT).shape == (4, 4)


def test_measurement_matrix_shapes_are_consistent():
    for model, m in ((PositionMeasurement(3.0), 2), (PositionVelocityMeasurement(3.0, 0.5), 4)):
        assert model.observation_matrix().shape == (m, 4)
        assert model.noise_covariance().shape == (m, m)


def test_process_noise_is_symmetric_psd():
    assert _is_symmetric_psd(ConstantAccelerationModel2D(0.2).process_noise(DT))


def test_covariance_stays_symmetric_psd():
    kf = _make_filter()
    gps = PositionMeasurement(3.0)
    for i in range(200):
        kf.predict(DT, control=np.zeros(2))
        if (i + 1) % 10 == 0:
            kf.update(np.array([0.0, 0.0]), gps)
        assert _is_symmetric_psd(kf.covariance)


def test_converges_on_constant_velocity_track():
    rng = np.random.default_rng(1)
    kf = _make_filter()
    gps = PositionMeasurement(3.0)
    velocity = np.array([2.0, -1.0])
    truth = np.zeros(2)
    for i in range(300):
        truth = truth + velocity * DT
        kf.predict(DT, control=np.zeros(2))
        if (i + 1) % 10 == 0:
            kf.update(truth + rng.normal(0.0, 3.0, 2), gps)
    assert np.linalg.norm(kf.state[:2] - truth) < 3.0
    assert np.allclose(kf.state[2:], velocity, atol=0.3)


def test_demo_filter_beats_raw_gps(capsys):
    from kalman_core import _run_demo
    _run_demo(seed=7)
    raw, filtered = (float(line.split(":")[1].split()[0])
                     for line in capsys.readouterr().out.splitlines())
    assert filtered < raw


def test_update_rejects_inconsistent_shapes():
    with pytest.raises(ValueError):
        _make_filter().update(np.zeros(3), PositionMeasurement(3.0))
