"""GPS-only position tracking on top of the Kalman filter core."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from geo_utils import LocalTangentPlane
from kalman_core import (ConstantAccelerationModel2D, KalmanFilter,
                         MotionModel, PositionMeasurement)


@dataclass(frozen=True)
class GpsFix:
    """A single GPS position fix."""

    timestamp: float      # seconds, any monotonic epoch (e.g. UNIX time)
    latitude_deg: float
    longitude_deg: float
    hdop: float


PlaneFactory = Callable[[float, float], LocalTangentPlane]


class PositionTracker:
    """
    Feeds GPS fixes into a Kalman filter and exposes the smoothed position.

    The first fix defines the local plane origin and initializes the filter.
    Every following fix runs predict(dt) and update(). Fixes with dt <= 0
    (duplicates and out-of-order fixes) are skipped. The class knows nothing
    about sockets or threads, so it can be driven by synthetic or recorded data.
    """

    DEFAULT_SIGMA_ACC = 0.5       # process noise: unmodeled acceleration, m/s^2
    INITIAL_SIGMA_VEL = 5.0       # initial velocity uncertainty, m/s

    def __init__(self,
                 motion_model: Optional[MotionModel] = None,
                 plane_factory: PlaneFactory = LocalTangentPlane) -> None:
        self._motion_model = motion_model or ConstantAccelerationModel2D(self.DEFAULT_SIGMA_ACC)
        self._plane_factory = plane_factory
        self._plane: Optional[LocalTangentPlane] = None
        self._filter: Optional[KalmanFilter] = None
        self._last_timestamp: Optional[float] = None

    @property
    def is_initialized(self) -> bool:
        return self._filter is not None

    @property
    def position_enu(self) -> tuple[float, float]:
        """Filtered (east, north) position in meters from the origin."""
        state = self._require_filter().state
        return float(state[0]), float(state[1])

    @property
    def velocity_enu(self) -> tuple[float, float]:
        """Filtered (east, north) velocity in m/s."""
        state = self._require_filter().state
        return float(state[2]), float(state[3])

    @property
    def position_geodetic(self) -> tuple[float, float]:
        """Filtered position as (latitude_deg, longitude_deg)."""
        east, north = self.position_enu
        assert self._plane is not None
        return self._plane.to_geodetic(east, north)

    def process(self, fix: GpsFix) -> bool:
        """Consume a fix. Returns False if it was skipped (dt <= 0)."""
        if self._filter is None or self._plane is None or self._last_timestamp is None:
            self._initialize(fix)
            return True

        dt = fix.timestamp - self._last_timestamp
        if dt <= 0:
            return False

        east, north = self._plane.to_local(fix.latitude_deg, fix.longitude_deg)
        self._filter.predict(dt)
        self._filter.update(np.array([east, north]), PositionMeasurement.from_hdop(fix.hdop))
        self._last_timestamp = fix.timestamp
        return True

    def _initialize(self, fix: GpsFix) -> None:
        self._plane = self._plane_factory(fix.latitude_deg, fix.longitude_deg)
        sigma_pos = PositionMeasurement.from_hdop(fix.hdop).noise_covariance()[0, 0]
        covariance = np.diag([sigma_pos, sigma_pos,
                              self.INITIAL_SIGMA_VEL ** 2, self.INITIAL_SIGMA_VEL ** 2])
        self._filter = KalmanFilter(self._motion_model, np.zeros(4), covariance)
        self._last_timestamp = fix.timestamp

    def _require_filter(self) -> KalmanFilter:
        if self._filter is None:
            raise RuntimeError("tracker has not received a fix yet")
        return self._filter
