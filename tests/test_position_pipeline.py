import threading

import pytest

pytest.importorskip("pynmea2")

from frame_adapter import FrameToGpsFixAdapter  # noqa: E402
from position_pipeline import PolledPoint, PositionPipeline  # noqa: E402
from tests.test_pipeline import ORIGIN_LAT, ORIGIN_LON, _epoch_packet  # noqa: E402
from tests.test_parser_frames import _SequenceSocket, build_packet, nmea, parser  # noqa: E402,F401
from tracker import GpsFix, PositionTracker  # noqa: E402


class _RecordingConsumer:
    """
    Consumer stub that remembers every fix it receives.

    Its position is (number of applied fixes, 0), so every applied fix gets its own position.
    Fixes whose index is in skip_indexes are reported as not applied.
    """

    def __init__(self, skip_indexes=()) -> None:
        self.fixes = []
        self._skip = set(skip_indexes)
        self._applied = 0

    def process(self, fix: GpsFix) -> bool:
        index = len(self.fixes)
        self.fixes.append(fix)
        if index in self._skip:
            return False
        self._applied += 1
        return True

    @property
    def position_enu(self) -> tuple:
        return float(self._applied), 0.0


def _packet(second: int) -> bytes:
    # a fix moving north by about 1.1e-5 degrees (about 1.2 m) per second
    return _epoch_packet(second, ORIGIN_LAT + 1e-5 * second, ORIGIN_LON)


def _gga_only_packet(second: int) -> bytes:
    """An epoch with a timestamp but without RMC: no date and no coordinates."""
    zeros = ("0", "0", "0")
    gga = f"GPGGA,1200{second:02d}.00,,,,,0,00,,,M,,M,,"
    return build_packet([gga], zeros, zeros, zeros)


class _Rig:
    """Parser + pipeline over a scripted packet sequence."""

    def __init__(self, parser, packets, consumer=None) -> None:
        parser.connection_mgr.m_socket = _SequenceSocket(packets)
        self.parser = parser
        self.lock = threading.Lock()
        self.consumer = consumer or _RecordingConsumer()
        self.pipeline = PositionPipeline(parser, FrameToGpsFixAdapter(), self.consumer, self.lock)

    def receive(self, count: int) -> None:
        for _ in range(count):
            self.parser.parser_gps(self.lock)

    def timestamps(self) -> list:
        return [fix.timestamp for fix in self.consumer.fixes]


def test_first_poll_returns_all_complete_frames_in_order(parser):
    rig = _Rig(parser, [_packet(t) for t in range(6)])
    rig.receive(6)

    points = rig.pipeline.poll()
    assert len(points) == 5  # the 6th frame is still incomplete
    assert all(isinstance(p, PolledPoint) and p.applied for p in points)
    assert [p.fix for p in points] == rig.consumer.fixes
    timestamps = rig.timestamps()
    assert timestamps == sorted(timestamps) and len(set(timestamps)) == 5
    assert [fix.latitude_deg for fix in rig.consumer.fixes] == pytest.approx(
        [ORIGIN_LAT + 1e-5 * t for t in range(5)], abs=1e-7)


def test_repeated_poll_without_new_data_returns_empty_list(parser):
    rig = _Rig(parser, [_packet(t) for t in range(4)])
    rig.receive(4)

    assert len(rig.pipeline.poll()) == 3
    assert rig.pipeline.poll() == []
    assert len(rig.consumer.fixes) == 3


def test_new_second_gives_exactly_one_new_point(parser):
    rig = _Rig(parser, [_packet(t) for t in range(5)])
    rig.receive(4)
    assert len(rig.pipeline.poll()) == 3

    rig.receive(1)
    new_points = rig.pipeline.poll()
    assert len(new_points) == 1 and new_points[0].applied
    assert new_points[0].fix == rig.consumer.fixes[-1]
    assert len(rig.consumer.fixes) == 4
    timestamps = rig.timestamps()
    assert timestamps == sorted(timestamps) and len(set(timestamps)) == 4


def test_frame_without_coordinates_is_skipped_without_breaking_the_rest(parser):
    packets = [_packet(0), _packet(1), _gga_only_packet(2), _packet(3), _packet(4), _packet(5)]
    rig = _Rig(parser, packets)
    rig.receive(6)

    assert [p.applied for p in rig.pipeline.poll()] == [True] * 4  # frames 0, 1, 3, 4; frame 2 is skipped
    timestamps = rig.timestamps()
    assert timestamps == sorted(timestamps)
    assert [round(b - a) for a, b in zip(timestamps, timestamps[1:])] == [1, 2, 1]


def test_each_point_has_its_own_position(parser):
    tracker = PositionTracker()
    rig = _Rig(parser, [_packet(t) for t in range(6)], consumer=tracker)
    rig.receive(6)

    points = rig.pipeline.poll()

    assert len(points) == 5
    norths = [p.position[1] for p in points]
    assert len(set(norths)) == 5          # not one and the same position for all points
    assert norths == sorted(norths)       # the track goes north, positions follow the order
    assert points[-1].position == tracker.position_enu


def test_skipped_point_is_reported_with_the_current_position(parser):
    rig = _Rig(parser, [_packet(t) for t in range(5)], consumer=_RecordingConsumer(skip_indexes={2}))
    rig.receive(5)

    points = rig.pipeline.poll()

    assert [p.applied for p in points] == [True, True, False, True]
    assert points[2].position == points[1].position   # unchanged by the skipped fix
    assert points[3].position != points[2].position
