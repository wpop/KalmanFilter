"""End-to-end test: synthetic UDP packets -> Parser -> get_data -> adapter -> PositionTracker."""

import threading

import numpy as np
import pytest

pytest.importorskip("pynmea2")

from frame_adapter import FrameToGpsFixAdapter  # noqa: E402
from geo_utils import LocalTangentPlane  # noqa: E402
from tests.test_parser_frames import _SequenceSocket, build_packet, parser  # noqa: E402,F401
from tracker import PositionTracker  # noqa: E402

ORIGIN_LAT, ORIGIN_LON = 55.76, 37.57
VELOCITY_E, VELOCITY_N = 2.0, 1.0   # true constant velocity, m/s
NOISE_SIGMA_M = 0.5                 # GPS noise injected into the packets
EPOCHS = 6                          # 6 epochs -> 5 complete frames + 1 incomplete


def _nmea_coordinate(value: float, degree_digits: int) -> str:
    """Decimal degrees (positive) -> NMEA ddmm.mmmmm / dddmm.mmmmm."""
    degrees = int(value)
    return f"{degrees:0{degree_digits}d}{(value - degrees) * 60:08.5f}"


def _epoch_packet(second: int, lat: float, lon: float) -> bytes:
    hhmmss = f"1200{second:02d}"
    lat_s, lon_s = _nmea_coordinate(lat, 2), _nmea_coordinate(lon, 3)
    sentences = [
        f"GPRMC,{hhmmss}.00,A,{lat_s},N,{lon_s},E,3.0,63.0,121026,,,A",
        f"GPGGA,{hhmmss}.00,{lat_s},N,{lon_s},E,1,08,1.0,150.0,M,14.0,M,,",
        "GPGSA,A,3,01,02,03,,,,,,,,,,1.8,1.0,1.4",
    ]
    zeros = ("0", "0", "0")
    return build_packet(sentences, zeros, zeros, zeros)


def test_packets_to_filtered_positions(parser):
    plane = LocalTangentPlane(ORIGIN_LAT, ORIGIN_LON)
    rng = np.random.default_rng(11)
    truth = [(VELOCITY_E * t, VELOCITY_N * t) for t in range(EPOCHS)]

    packets = []
    for t, (east, north) in enumerate(truth):
        noisy = np.array([east, north]) + rng.normal(0.0, NOISE_SIGMA_M, 2)
        packets.append(_epoch_packet(t, *plane.to_geodetic(*noisy)))

    parser.connection_mgr.m_socket = _SequenceSocket(packets)
    lock = threading.Lock()
    for _ in packets:
        parser.parser_gps(lock)

    frames = parser.get_data(lock)
    # the last, still incomplete frame must not be returned
    assert len(frames) == EPOCHS - 1

    adapter = FrameToGpsFixAdapter()
    fixes = [adapter.convert(frame) for frame in frames]
    assert all(fix is not None for fix in fixes)

    # get_data returns frames newest first; the consumer restores chronological order
    timestamps = [fix.timestamp for fix in fixes]
    assert timestamps == sorted(timestamps, reverse=True)
    fixes.reverse()
    assert [b.timestamp - a.timestamp for a, b in zip(fixes, fixes[1:])] == [1.0] * (EPOCHS - 2)

    tracker = PositionTracker()
    errors = []
    for fix, (east, north) in zip(fixes, truth):
        assert tracker.process(fix)
        # the tracker's origin is its first fix, so compare in the common frame
        filtered = np.array(plane.to_local(*tracker.position_geodetic))
        errors.append(np.linalg.norm(filtered - [east, north]))

    assert max(errors) < 2.5
