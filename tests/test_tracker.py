import numpy as np
import pytest

from geo_utils import LocalTangentPlane
from tracker import GpsFix, PositionTracker

LAT0, LON0 = 55.76, 37.57


def test_first_fix_sets_origin_and_initializes():
    tracker = PositionTracker()
    assert not tracker.is_initialized
    with pytest.raises(RuntimeError):
        tracker.position_enu
    assert tracker.process(GpsFix(0.0, LAT0, LON0, 1.0))
    assert tracker.is_initialized
    assert tracker.position_enu == (0.0, 0.0)
    assert tracker.position_geodetic == pytest.approx((LAT0, LON0))


def test_duplicate_and_out_of_order_fixes_are_skipped():
    tracker = PositionTracker()
    tracker.process(GpsFix(10.0, LAT0, LON0, 1.0))
    assert tracker.process(GpsFix(11.0, LAT0 + 1e-4, LON0, 1.0))
    before = tracker.position_enu
    assert not tracker.process(GpsFix(11.0, LAT0 + 5e-4, LON0, 1.0))  # duplicate
    assert not tracker.process(GpsFix(9.0, LAT0 + 5e-4, LON0, 1.0))   # out of order
    assert tracker.position_enu == before


def test_smooths_noisy_track():
    rng = np.random.default_rng(3)
    plane = LocalTangentPlane(LAT0, LON0)
    # The tracker's own origin is the first noisy fix, so compare in a common frame
    tracker = PositionTracker()
    raw_err, filt_err = [], []
    for t in range(120):
        true_e, true_n = 1.5 * t, 0.5 * t  # constant velocity, 1 Hz fixes
        noisy = np.array([true_e, true_n]) + rng.normal(0.0, 3.0, 2)
        lat, lon = plane.to_geodetic(*noisy)
        tracker.process(GpsFix(float(t), lat, lon, 1.0))
        if t >= 20:
            raw_err.append(np.linalg.norm(noisy - [true_e, true_n]))
            filt_err.append(np.linalg.norm(np.array(plane.to_local(*tracker.position_geodetic)) - [true_e, true_n]))
    rmse = lambda e: float(np.sqrt(np.mean(np.square(e))))
    assert rmse(filt_err) < rmse(raw_err)
