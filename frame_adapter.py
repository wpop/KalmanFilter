"""Conversion of parsed sensor frames into GPS fixes for the position tracker."""

from __future__ import annotations

import datetime
import logging
import math
from typing import Optional

from Sensor_Data import Sensors
from tracker import GpsFix


class FrameToGpsFixAdapter:
    """
    Turns a Sensors frame into a GpsFix.

    Returns None (and logs at debug level) if the frame has no time, date, latitude or
    longitude, because such a frame cannot be placed on the map or on the time axis.

    The fix timestamp is the UNIX time in seconds of frame.date combined with frame.time
    (frame.time is UTC; a naive time is treated as UTC).

    HDOP handling: the position is still valid when HDOP is missing, so the fix is not
    dropped. If frame.hdop is missing, not a number, not finite or not positive, default_hdop
    is used instead. The default of 5.0 is the upper edge of the "good" HDOP range
    (1 ideal, 1-2 excellent, 2-5 good, 5-10 moderate); it makes the tracker trust a fix of
    unknown quality noticeably less than a typical one, without ignoring it. Tune it on
    real recordings.
    """

    DEFAULT_HDOP = 5.0

    def __init__(self, default_hdop: float = DEFAULT_HDOP) -> None:
        if default_hdop <= 0:
            raise ValueError("default_hdop must be positive")
        self._default_hdop = default_hdop

    def convert(self, frame: Sensors) -> Optional[GpsFix]:
        missing = [name for name, value in (("time", frame.time), ("date", frame.date),
                                            ("latitude", frame.latitude_deg),
                                            ("longitude", frame.longitude_deg)) if value is None]
        if missing:
            logging.debug(f'Skipping frame without {", ".join(missing)}')
            return None

        return GpsFix(timestamp=self._timestamp(frame.date, frame.time),
                      latitude_deg=float(frame.latitude_deg),
                      longitude_deg=float(frame.longitude_deg),
                      hdop=self._hdop(frame.hdop))

    @staticmethod
    def _timestamp(date: datetime.date, time: datetime.time) -> float:
        if time.tzinfo is None:
            time = time.replace(tzinfo=datetime.timezone.utc)
        return datetime.datetime.combine(date, time).timestamp()

    def _hdop(self, raw) -> float:
        try:
            hdop = float(raw)
        except (TypeError, ValueError):
            hdop = math.nan
        if not math.isfinite(hdop) or hdop <= 0:
            logging.debug(f'Invalid hdop {raw!r}, using default {self._default_hdop}')
            return self._default_hdop
        return hdop
