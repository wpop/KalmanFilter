"""Replay a recorded UDP stream through the parser, the pipeline and the tracker.

The input is a file written by tools/record_udp.py: one packet per line,

    <unix_time_seconds>\t<payload with backslash escapes>

Packets are fed to a Parser through a socket stub (the Parser is created on port 0, so no
real port is used). After every packet PositionPipeline.poll() is called and each new point
is printed as a table row; a summary is printed at the end.

Usage:
    python tools/replay.py RECORDING [-v]
"""

from __future__ import annotations

import argparse
import codecs
import datetime
import logging
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional, TextIO

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # allow running as a script

from frame_adapter import FrameToGpsFixAdapter  # noqa: E402
from GPS_Parser import Parser  # noqa: E402
from position_pipeline import PositionPipeline  # noqa: E402
from Sensor_Data import Sensors  # noqa: E402
from tracker import GpsFix, PositionTracker  # noqa: E402

HEADER = f"{'time (UTC)':<10}  {'latitude':>11}  {'longitude':>11}  {'hdop':>5}  {'east, m':>9}  {'north, m':>9}  status"


def unescape_payload(text: str) -> bytes:
    """Inverse of tools.record_udp.escape_payload."""
    return codecs.decode(text, "unicode_escape").encode("latin-1")


def read_recording(path: Path) -> Iterator[tuple[float, bytes]]:
    """Yield (receive_time, packet) for every line of a recording."""
    with path.open(encoding="ascii") as handle:
        for number, line in enumerate(handle, start=1):
            line = line.rstrip("\n")
            if not line:
                continue
            received, separator, payload = line.partition("\t")
            if not separator:
                raise ValueError(f"{path}:{number}: expected '<time><TAB><payload>'")
            yield float(received), unescape_payload(payload)


class _ReplaySocket:
    """Socket stub returning the next prepared packet on every recvfrom call."""

    def __init__(self) -> None:
        self.next_packet = b""

    def recvfrom(self, _size: int):
        return self.next_packet, ("127.0.0.1", 0)


class _TrackingConverter:
    """Wraps a converter and remembers which frames it rejected (each frame counted once)."""

    def __init__(self, inner: FrameToGpsFixAdapter) -> None:
        self._inner = inner
        self.rejected: dict[int, Sensors] = {}  # holds the frames, so ids stay unique

    def convert(self, frame: Sensors) -> Optional[GpsFix]:
        fix = self._inner.convert(frame)
        if fix is None:
            self.rejected[id(frame)] = frame
        return fix


@dataclass(frozen=True)
class ReplaySummary:
    packets: int
    frames: int
    accepted: int   # points applied by the tracker
    skipped: int    # points skipped by the tracker plus frames rejected by the adapter


def replay(path: Path, out: TextIO = sys.stdout) -> ReplaySummary:
    """Run a recording through the whole chain, print the table and return the summary."""
    parser = Parser(0)  # port 0: the OS picks a free port
    real_socket = parser.connection_mgr.m_socket
    stub = _ReplaySocket()
    parser.connection_mgr.m_socket = stub
    try:
        converter = _TrackingConverter(FrameToGpsFixAdapter())
        pipeline = PositionPipeline(parser, converter, PositionTracker(), threading.Lock())
        lock = threading.Lock()

        packets = frames = accepted = tracker_skipped = 0
        newest_frame = None
        print(HEADER, file=out)
        for _received, packet in read_recording(path):
            stub.next_packet = packet
            parser.parser_gps(lock)
            packets += 1
            if parser.frames and parser.frames[0] is not newest_frame:
                newest_frame = parser.frames[0]
                frames += 1
            for point in pipeline.poll():
                accepted += point.applied
                tracker_skipped += not point.applied
                print(_format_row(point.fix, point.applied, point.position), file=out)

        summary = ReplaySummary(packets, frames, accepted, tracker_skipped + len(converter.rejected))
        print(f"\nPackets: {summary.packets}, frames: {summary.frames}, "
              f"points accepted: {summary.accepted}, skipped: {summary.skipped}", file=out)
        return summary
    finally:
        real_socket.close()


def _format_row(fix: GpsFix, applied: bool, position: tuple[float, float]) -> str:
    when = datetime.datetime.fromtimestamp(fix.timestamp, datetime.timezone.utc).strftime("%H:%M:%S")
    return (f"{when:<10}  {fix.latitude_deg:>11.6f}  {fix.longitude_deg:>11.6f}  {fix.hdop:>5.1f}  "
            f"{position[0]:>9.2f}  {position[1]:>9.2f}  {'ok' if applied else 'skipped'}")


def main() -> None:
    args = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    args.add_argument("recording", type=Path, help="file written by tools/record_udp.py")
    args.add_argument("-v", "--verbose", action="store_true", help="show parser debug logging")
    options = args.parse_args()
    # GPS_Parser configures DEBUG logging on import; keep the table readable by default
    logging.getLogger().setLevel(logging.DEBUG if options.verbose else logging.WARNING)
    replay(options.recording)


if __name__ == "__main__":
    main()
