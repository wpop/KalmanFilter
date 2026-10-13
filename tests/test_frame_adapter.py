import datetime
import logging

import pytest

from frame_adapter import FrameToGpsFixAdapter
from Sensor_Data import Sensors

UTC = datetime.timezone.utc


def _frame(**overrides) -> Sensors:
    frame = Sensors()
    frame.time = datetime.time(12, 0, 5, tzinfo=UTC)
    frame.date = datetime.date(2026, 10, 12)
    frame.latitude_deg = 55.76
    frame.longitude_deg = -37.57
    frame.hdop = "1.2"
    for name, value in overrides.items():
        setattr(frame, name, value)
    return frame


def test_converts_complete_frame():
    fix = FrameToGpsFixAdapter().convert(_frame())
    expected = datetime.datetime(2026, 10, 12, 12, 0, 5, tzinfo=UTC).timestamp()
    assert (fix.timestamp, fix.latitude_deg, fix.longitude_deg, fix.hdop) == (expected, 55.76, -37.57, 1.2)


@pytest.mark.parametrize("field", ["time", "date", "latitude_deg", "longitude_deg"])
def test_frame_with_missing_field_gives_none_and_debug_log(field, caplog):
    with caplog.at_level(logging.DEBUG):
        assert FrameToGpsFixAdapter().convert(_frame(**{field: None})) is None
    assert any(field.split("_")[0] in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize("hdop", [None, "", "abc", "nan", "0", "-1.0"])
def test_invalid_hdop_falls_back_to_default(hdop):
    fix = FrameToGpsFixAdapter(default_hdop=4.0).convert(_frame(hdop=hdop))
    assert fix is not None and fix.hdop == 4.0
