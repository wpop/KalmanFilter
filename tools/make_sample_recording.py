"""Generate a small synthetic recording for tools/replay.py.

The recording has 11 one-second epochs (10 seconds of motion) of a rover moving at a
constant velocity with a little GPS noise, in the format written by tools/record_udp.py.
The fifth epoch has no GSA sentence, so its HDOP is missing and the adapter fallback shows.

Usage:
    python tools/make_sample_recording.py [-o tests/data/sample_recording.log]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # allow running as a script

from geo_utils import LocalTangentPlane  # noqa: E402
from tools.record_udp import escape_payload  # noqa: E402

ORIGIN_LAT, ORIGIN_LON = 55.76, 37.57
VELOCITY_EAST, VELOCITY_NORTH = 2.0, 1.0   # m/s
NOISE_SIGMA_M = 1.0
EPOCHS = 11
EPOCH_WITHOUT_HDOP = 5
START_UNIX_TIME = 1791806400.0             # 2026-10-12 12:00:00 UTC, matches the NMEA date below
SEED = 5


def nmea(body: str) -> str:
    """Wrap a sentence body into '$...*CS' with a valid checksum; ';' replaces ',' as on the phone."""
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}".replace(",", ";")


def nmea_coordinate(value: float, degree_digits: int) -> str:
    """Positive decimal degrees -> NMEA ddmm.mmmmm / dddmm.mmmmm."""
    degrees = int(value)
    return f"{degrees:0{degree_digits}d}{(value - degrees) * 60:08.5f}"


def build_packet(sentences: list[str], accel: tuple[str, str, str], sent_at: str) -> bytes:
    """Wire format: a leading field, then 4-field groups ([1, NMEA, pad, pad], [3, x, y, z])."""
    fields = [sent_at]
    for sentence in sentences:
        fields += ["1", nmea(sentence), "0", "0"]
    fields += ["3", *accel]
    return (",".join(fields) + "\n").encode("utf-8")


def make_packets() -> list[tuple[float, bytes]]:
    plane = LocalTangentPlane(ORIGIN_LAT, ORIGIN_LON)
    rng = np.random.default_rng(SEED)
    packets = []
    for second in range(EPOCHS):
        east, north = VELOCITY_EAST * second, VELOCITY_NORTH * second
        lat, lon = plane.to_geodetic(*(np.array([east, north]) + rng.normal(0.0, NOISE_SIGMA_M, 2)))
        lat_s, lon_s = nmea_coordinate(lat, 2), nmea_coordinate(lon, 3)
        hhmmss = f"1200{second:02d}"
        sentences = [
            f"GPRMC,{hhmmss}.00,A,{lat_s},N,{lon_s},E,4.4,63.0,121026,,,A",
            f"GPGGA,{hhmmss}.00,{lat_s},N,{lon_s},E,1,08,1.0,150.0,M,14.0,M,,",
        ]
        if second != EPOCH_WITHOUT_HDOP:
            sentences.append("GPGSA,A,3,01,02,03,,,,,,,,,,1.8,1.0,1.4")
        sentences.append("GPGSV,1,1,02,01,40,083,46,02,17,308,41")
        received = START_UNIX_TIME + second
        packets.append((received, build_packet(sentences, ("0.10", "0.20", "9.81"), f"{received:.3f}")))
    return packets


def main() -> None:
    args = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    args.add_argument("-o", "--output", type=Path, default=Path("tests/data/sample_recording.log"))
    options = args.parse_args()
    options.output.parent.mkdir(parents=True, exist_ok=True)
    with options.output.open("w", encoding="ascii") as handle:
        for received, packet in make_packets():
            handle.write(f"{received:.6f}\t{escape_payload(packet)}\n")
    print(f"Wrote {EPOCHS} packets to {options.output}")


if __name__ == "__main__":
    main()
