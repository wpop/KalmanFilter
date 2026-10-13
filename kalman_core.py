"""
Reference implementation of a linear Kalman filter for a ground robot (rover).

State:          x = [east, north, v_east, v_north]^T   (position in meters and velocity in m/s, local ENU frame)
Control:        u = [a_east, a_north]^T                (acceleration measured by the IMU and rotated into ENU)
Measurements:   GPS position (+ optionally velocity from RMC)

Architecture (SOLID):
    * S - each class has one responsibility: motion model, measurement model, the filter itself.
    * O - new motion / measurement models are added by subclassing; the filter does not change.
    * L - any MotionModel / MeasurementModel implementation is interchangeable.
    * I - interfaces are narrow: a motion model has three methods, a measurement model has two.
    * D - KalmanFilter depends on abstractions (ABC), not on concrete matrices.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


# --------------------------------------------------------------------------- #
#  Motion models
# --------------------------------------------------------------------------- #
class MotionModel(ABC):
    """Abstraction of how the system evolves over time."""

    @property
    @abstractmethod
    def state_dim(self) -> int:
        """Dimension of the state vector."""

    @abstractmethod
    def transition(self, dt: float) -> np.ndarray:
        """State transition matrix F(dt)."""

    @abstractmethod
    def control(self, dt: float) -> np.ndarray:
        """Control matrix B(dt)."""

    @abstractmethod
    def process_noise(self, dt: float) -> np.ndarray:
        """Process noise covariance Q(dt)."""


class ConstantAccelerationModel2D(MotionModel):
    """
    Planar motion: acceleration comes from the IMU as a control input,
    and its inaccuracy is described by the process noise.

        e'  = e  + ve*dt + 0.5*ae*dt^2
        ve' = ve + ae*dt
    """

    def __init__(self, sigma_acc: float) -> None:
        if sigma_acc <= 0:
            raise ValueError("sigma_acc must be positive")
        self._sigma_acc = sigma_acc  # accelerometer noise std, m/s^2

    @property
    def state_dim(self) -> int:
        return 4

    def transition(self, dt: float) -> np.ndarray:
        # F holds dt (not velocity): position += velocity * dt
        f = np.eye(4)
        f[0, 2] = dt
        f[1, 3] = dt
        return f

    def control(self, dt: float) -> np.ndarray:
        # Acceleration contributes 0.5*dt^2 to position and dt to velocity
        half_dt2 = 0.5 * dt * dt
        return np.array([
            [half_dt2, 0.0],
            [0.0, half_dt2],
            [dt, 0.0],
            [0.0, dt],
        ])

    def process_noise(self, dt: float) -> np.ndarray:
        # Acceleration noise enters the state exactly through B, so Q = sigma^2 * B * B^T.
        # The result is symmetric and positive semi-definite by construction.
        b = self.control(dt)
        return (self._sigma_acc ** 2) * (b @ b.T)


# --------------------------------------------------------------------------- #
#  Measurement models
# --------------------------------------------------------------------------- #
class MeasurementModel(ABC):
    """Abstraction of what is observed and with what accuracy."""

    @abstractmethod
    def observation_matrix(self) -> np.ndarray:
        """Observation matrix H (m x n)."""

    @abstractmethod
    def noise_covariance(self) -> np.ndarray:
        """Measurement noise covariance R (m x m, consistent with H!)."""


class PositionMeasurement(MeasurementModel):
    """GPS provides position only (east, north)."""

    # Rough estimate: position std ~= HDOP * UERE. The UERE value is an assumption and
    # must be calibrated on real data (about 3-5 m for a typical smartphone).
    DEFAULT_UERE_M = 3.0

    def __init__(self, sigma_pos: float) -> None:
        if sigma_pos <= 0:
            raise ValueError("sigma_pos must be positive")
        self._sigma_pos = sigma_pos

    @classmethod
    def from_hdop(cls, hdop: float, uere: float = DEFAULT_UERE_M) -> "PositionMeasurement":
        """Build the measurement model from the HDOP already parsed from GSA."""
        return cls(sigma_pos=max(hdop, 1.0) * uere)

    def observation_matrix(self) -> np.ndarray:
        return np.eye(2, 4)

    def noise_covariance(self) -> np.ndarray:
        return np.eye(2) * self._sigma_pos ** 2


class PositionVelocityMeasurement(MeasurementModel):
    """GPS provides both position and velocity (e.g. from RMC, split into axes)."""

    def __init__(self, sigma_pos: float, sigma_vel: float) -> None:
        if sigma_pos <= 0 or sigma_vel <= 0:
            raise ValueError("standard deviations must be positive")
        self._sigma_pos = sigma_pos
        self._sigma_vel = sigma_vel

    def observation_matrix(self) -> np.ndarray:
        return np.eye(4)

    def noise_covariance(self) -> np.ndarray:
        return np.diag([self._sigma_pos ** 2, self._sigma_pos ** 2,
                        self._sigma_vel ** 2, self._sigma_vel ** 2])


# --------------------------------------------------------------------------- #
#  Results and the filter itself
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class UpdateReport:
    """Update step diagnostics, useful for rejecting GPS outliers."""

    innovation: np.ndarray  # innovation y = z - H x
    nis: float              # normalized innovation squared y^T S^-1 y (~ chi-square)


class KalmanFilter:
    """Linear Kalman filter: predict -> update cycle."""

    def __init__(self, motion_model: MotionModel,
                 initial_state: np.ndarray,
                 initial_covariance: np.ndarray) -> None:
        n = motion_model.state_dim
        x0 = np.asarray(initial_state, dtype=float).reshape(n, 1)
        p0 = np.asarray(initial_covariance, dtype=float)
        if p0.shape != (n, n):
            raise ValueError(f"P0 must be {n}x{n}")
        self._model = motion_model
        self._x = x0
        self._p = p0

    @property
    def state(self) -> np.ndarray:
        return self._x.ravel().copy()

    @property
    def covariance(self) -> np.ndarray:
        return self._p.copy()

    def predict(self, dt: float, control: np.ndarray | None = None) -> None:
        """Predict where the system moved in time dt and how much less certain we became."""
        if dt <= 0:
            raise ValueError("dt must be positive")
        f = self._model.transition(dt)
        q = self._model.process_noise(dt)

        self._x = f @ self._x
        if control is not None:
            b = self._model.control(dt)
            self._x += b @ np.asarray(control, dtype=float).reshape(-1, 1)

        self._p = f @ self._p @ f.T + q

    def update(self, measurement: np.ndarray, model: MeasurementModel) -> UpdateReport:
        """Correct: blend the prediction with the measurement in proportion to trust in each."""
        h = model.observation_matrix()
        r = model.noise_covariance()
        z = np.asarray(measurement, dtype=float).reshape(-1, 1)
        if h.shape[0] != z.shape[0] or r.shape != (h.shape[0], h.shape[0]):
            raise ValueError("Shapes of z, H and R are inconsistent")

        y = z - h @ self._x                     # innovation
        s = h @ self._p @ h.T + r               # innovation covariance
        # Kalman gain K = P H^T S^-1; solve a linear system instead of inverting explicitly
        k = np.linalg.solve(s, h @ self._p).T

        self._x = self._x + k @ y

        # Joseph form: keeps P symmetric and positive definite despite rounding errors
        i_kh = np.eye(self._p.shape[0]) - k @ h
        self._p = i_kh @ self._p @ i_kh.T + k @ r @ k.T
        self._p = 0.5 * (self._p + self._p.T)

        nis = (y.T @ np.linalg.solve(s, y)).item()  # 1x1 result -> plain number
        return UpdateReport(innovation=y.ravel(), nis=nis)


# --------------------------------------------------------------------------- #
#  Simulation demo: check that the filter really beats raw GPS
# --------------------------------------------------------------------------- #
def _run_demo(seed: int = 7) -> None:
    rng = np.random.default_rng(seed)

    dt_imu = 0.1          # IMU runs at 10 Hz
    gps_every = 10        # GPS arrives once per second
    steps = 600           # 60 seconds of motion
    sigma_gps = 3.0       # true GPS error, m
    sigma_acc = 0.2       # true accelerometer error, m/s^2

    # True trajectory: acceleration, turn (acceleration along north), braking
    true_acc = np.zeros((steps, 2))
    true_acc[:100, 0] = 0.5
    true_acc[200:300, 1] = 0.4
    true_acc[400:500, 0] = -0.4

    true_state = np.zeros((steps, 4))
    s = np.zeros(4)
    for i in range(steps):
        a = true_acc[i]
        s = np.array([s[0] + s[2] * dt_imu + 0.5 * a[0] * dt_imu ** 2,
                      s[1] + s[3] * dt_imu + 0.5 * a[1] * dt_imu ** 2,
                      s[2] + a[0] * dt_imu,
                      s[3] + a[1] * dt_imu])
        true_state[i] = s

    # The filter starts blind: position is off, velocity is unknown
    kf = KalmanFilter(
        motion_model=ConstantAccelerationModel2D(sigma_acc=sigma_acc),
        initial_state=np.array([5.0, -5.0, 0.0, 0.0]),
        initial_covariance=np.diag([25.0, 25.0, 4.0, 4.0]),
    )
    gps_model = PositionMeasurement(sigma_pos=sigma_gps)

    err_kf, err_gps = [], []
    for i in range(steps):
        imu = true_acc[i] + rng.normal(0.0, sigma_acc, 2)
        kf.predict(dt_imu, control=imu)

        if (i + 1) % gps_every == 0:
            gps = true_state[i, :2] + rng.normal(0.0, sigma_gps, 2)
            kf.update(gps, gps_model)
            err_gps.append(np.linalg.norm(gps - true_state[i, :2]))
            err_kf.append(np.linalg.norm(kf.state[:2] - true_state[i, :2]))

    # Skip the first 10 epochs: the filter is still converging from the wrong initial state
    rmse = lambda e: float(np.sqrt(np.mean(np.square(e[10:]))))
    print(f"Raw GPS RMSE : {rmse(err_gps):.2f} m")
    print(f"Filter RMSE  : {rmse(err_kf):.2f} m")


if __name__ == "__main__":
    _run_demo()
