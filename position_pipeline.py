"""Feeds new parser frames into the position tracker."""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Optional, Protocol, Sequence

from Sensor_Data import Sensors
from tracker import GpsFix


class FrameSource(Protocol):
    """Anything that returns completed frames, newest first (like Parser)."""

    def get_data(self, lock: AbstractContextManager) -> Sequence[Sensors]: ...


class FixConverter(Protocol):
    """Anything that turns a frame into a GpsFix or None (like FrameToGpsFixAdapter)."""

    def convert(self, frame: Sensors) -> Optional[GpsFix]: ...


class FixConsumer(Protocol):
    """Anything that consumes fixes and exposes the position (like PositionTracker)."""

    def process(self, fix: GpsFix) -> bool: ...

    @property
    def position_enu(self) -> tuple[float, float]: ...


@dataclass(frozen=True)
class PolledPoint:
    """The outcome of feeding one fix to the consumer."""

    fix: GpsFix
    applied: bool                   # False if the consumer skipped the fix
    position: tuple[float, float]   # (east, north) in meters right after processing this fix


class PositionPipeline:
    """
    Takes the frames that appeared since the previous poll and passes them to the tracker.

    Each poll() reads the frames from the source, restores chronological order (the source
    returns them newest first), skips frames whose timestamp is not newer than the last
    processed one, converts the rest into fixes and feeds them to the consumer. Frames the
    converter rejects (no time, date or coordinates) are skipped and do not stop the others.

    There are no sockets or threads here: the caller decides when to call poll().
    """

    def __init__(self, source: FrameSource, converter: FixConverter,
                 consumer: FixConsumer, lock: AbstractContextManager) -> None:
        self._source = source
        self._converter = converter
        self._consumer = consumer
        self._lock = lock
        self._last_timestamp: Optional[float] = None  # timestamp of the last processed frame

    def poll(self) -> list[PolledPoint]:
        """
        Process new frames and return one PolledPoint per new fix, oldest first.

        applied is what PositionTracker.process returns: True if the fix was applied, False
        if the tracker skipped it (dt <= 0). A skipped fix is still reported, and its position
        is the tracker's current position. The position is read right after process(), and the
        first fix always initializes the tracker, so it is always available; the consumer's
        own error (RuntimeError for an uninitialized tracker) is not caught. Frames rejected by
        the converter and frames processed earlier produce no entry. An empty list means
        there was nothing new.
        """
        results: list[PolledPoint] = []
        for frame in reversed(list(self._source.get_data(self._lock))):
            fix = self._converter.convert(frame)
            if fix is None:
                continue
            if self._last_timestamp is not None and fix.timestamp <= self._last_timestamp:
                continue
            applied = self._consumer.process(fix)
            results.append(PolledPoint(fix, applied, self._consumer.position_enu))
            self._last_timestamp = fix.timestamp
        return results
