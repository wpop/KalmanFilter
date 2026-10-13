"""Frame assembly tests for Parser.parser_gps on synthetic UDP packets."""

import datetime
import logging
import threading
from dataclasses import dataclass
from typing import Sequence

import pytest

pytest.importorskip("pynmea2")

from GPS_Parser import Parser  # noqa: E402

UTC = datetime.timezone.utc


def nmea(body: str) -> str:
    """Wrap a sentence body in '$...*CS' with a valid checksum, ';' instead of ','."""
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}".replace(",", ";")


def build_packet(sentences: Sequence[str], accel, gyro, mag, sent_at: str = "1760000000") -> bytes:
    """
    Build a UDP packet in the phone's wire format: the first field is a timestamp,
    followed by 4-field groups. Type 1 is [1, NMEA, pad, pad]; types 3, 4 and 5
    are [type, x, y, z] for accelerometer, gyroscope and magnetometer.
    """
    fields = [sent_at]
    for sentence in sentences:
        fields += ["1", nmea(sentence), "0", "0"]
    for kind, xyz in ((3, accel), (4, gyro), (5, mag)):
        fields += [str(kind), *xyz]
    return (",".join(fields) + "\n").encode("utf-8")


@dataclass(frozen=True)
class Epoch:
    """Everything a single epoch carries, with expected parsed values."""

    hhmmss: str
    lat_nmea: str
    lon_nmea: str
    speed_knots: float
    altitude: float
    pdop: str
    hdop: str
    vdop: str
    accel: tuple
    gyro: tuple
    mag: tuple

    def packet(self) -> bytes:
        sentences = [
            f"GPRMC,{self.hhmmss}.00,A,{self.lat_nmea},N,{self.lon_nmea},E,{self.speed_knots},90.0,121026,,,A",
            f"GPGGA,{self.hhmmss}.00,{self.lat_nmea},N,{self.lon_nmea},E,1,08,0.9,{self.altitude},M,14.0,M,,",
            f"GPGSA,A,3,01,02,03,,,,,,,,,,{self.pdop},{self.hdop},{self.vdop}",
            "GPGSV,1,1,02,01,40,083,46,02,17,308,41",
        ]
        return build_packet(sentences, self.accel, self.gyro, self.mag)


EPOCH_1 = Epoch("120000", "5545.6000", "03734.2000", 1.5, 150.5, "1.8", "1.2", "1.4",
                ("0.11", "0.12", "9.81"), ("0.01", "0.02", "0.03"), ("21.0", "-5.0", "40.0"))
EPOCH_2 = Epoch("120001", "5546.1000", "03735.1000", 2.5, 151.5, "2.8", "1.7", "2.4",
                ("0.21", "0.22", "9.79"), ("0.04", "0.05", "0.06"), ("22.0", "-6.0", "41.0"))

LAT_1, LON_1 = "55°45′36.0000″N", "37°34′12.0000″E"
LAT_2, LON_2 = "55°46′06.0000″N", "37°35′06.0000″E"


class _SequenceSocket:
    """Socket stub that returns the prepared packets one by one."""

    def __init__(self, packets: Sequence[bytes]) -> None:
        self._packets = list(packets)

    def recvfrom(self, _size: int):
        return self._packets.pop(0), ("127.0.0.1", 12345)


@pytest.fixture
def parser():
    parser = Parser(0)  # port 0: the OS picks a free port
    real_socket = parser.connection_mgr.m_socket
    yield parser
    real_socket.close()


def feed(parser: Parser, packets: Sequence[bytes], caplog) -> None:
    """Run parser_gps once per packet and make sure nothing was swallowed as an error."""
    parser.connection_mgr.m_socket = _SequenceSocket(packets)
    lock = threading.Lock()
    with caplog.at_level(logging.ERROR):
        for _ in packets:
            parser.parser_gps(lock)
    assert [r.getMessage() for r in caplog.records if r.levelno >= logging.ERROR] == []


def test_single_frame_has_all_fields(parser, caplog):
    feed(parser, [EPOCH_1.packet()], caplog)

    frame = parser.frames[0]
    assert frame.time == datetime.time(12, 0, 0, tzinfo=UTC)
    assert frame.latitude == LAT_1
    assert frame.longitude == LON_1
    assert frame.altitude == 150.5
    assert frame.pdop == "1.8"
    assert frame.hdop == "1.2"
    assert frame.vdop == "1.4"
    assert frame.vel_knot == 1.5
    assert frame.sv_prn_num_gps == ["01", "02"]
    assert (frame.acceleration_x, frame.acceleration_y, frame.acceleration_z) == EPOCH_1.accel
    assert (frame.gyr_x, frame.gyr_y, frame.gyr_z) == EPOCH_1.gyro
    assert (frame.dir_x, frame.dir_y, frame.dir_z) == EPOCH_1.mag


def test_two_epochs_land_in_separate_consistent_frames(parser, caplog):
    feed(parser, [EPOCH_1.packet(), EPOCH_2.packet()], caplog)

    # frames are stored newest first
    newest, oldest = parser.frames[0], parser.frames[1]
    for frame, epoch, lat, lon, second in ((oldest, EPOCH_1, LAT_1, LON_1, 0),
                                           (newest, EPOCH_2, LAT_2, LON_2, 1)):
        assert frame.time == datetime.time(12, 0, second, tzinfo=UTC)
        assert frame.latitude == lat
        assert frame.longitude == lon
        assert frame.altitude == epoch.altitude
        assert frame.pdop == epoch.pdop
        assert frame.hdop == epoch.hdop
        assert frame.vdop == epoch.vdop
        assert frame.vel_knot == epoch.speed_knots
        assert (frame.acceleration_x, frame.acceleration_y, frame.acceleration_z) == epoch.accel
        assert (frame.gyr_x, frame.gyr_y, frame.gyr_z) == epoch.gyro
        assert (frame.dir_x, frame.dir_y, frame.dir_z) == epoch.mag
        assert frame.sv_prn_num_gps == ["01", "02"]


def _split_epoch_packets(epoch: Epoch) -> list:
    """Split one epoch into two packets: position sentences first, then DOP and IMU."""
    sentences = [
        f"GPRMC,{epoch.hhmmss}.00,A,{epoch.lat_nmea},N,{epoch.lon_nmea},E,{epoch.speed_knots},90.0,121026,,,A",
        f"GPGGA,{epoch.hhmmss}.00,{epoch.lat_nmea},N,{epoch.lon_nmea},E,1,08,0.9,{epoch.altitude},M,14.0,M,,",
        f"GPGSA,A,3,01,02,03,,,,,,,,,,{epoch.pdop},{epoch.hdop},{epoch.vdop}",
    ]
    zeros = ("0", "0", "0")
    return [build_packet(sentences[:2], zeros, zeros, zeros),
            build_packet(sentences[2:], epoch.accel, epoch.gyro, epoch.mag)]


@pytest.mark.xfail(strict=True, reason=(
    "Known bug: a new frame is created at the start of parser_gps from the RMC time of the "
    "previous packet, so frames lag by one packet. When an epoch spans two packets, its "
    "second packet (GSA/IMU) is written into the next epoch's frame, and a trailing empty "
    "frame is created."))
def test_epoch_split_across_two_packets_stays_in_one_frame(parser, caplog):
    feed(parser, _split_epoch_packets(EPOCH_1) + _split_epoch_packets(EPOCH_2), caplog)

    assert len(parser.frames) == 2
    newest, oldest = parser.frames[0], parser.frames[1]
    for frame, epoch, second in ((oldest, EPOCH_1, 0), (newest, EPOCH_2, 1)):
        assert frame.time == datetime.time(12, 0, second, tzinfo=UTC)
        assert frame.altitude == epoch.altitude
        assert frame.hdop == epoch.hdop
        assert (frame.acceleration_x, frame.acceleration_y, frame.acceleration_z) == epoch.accel
        assert (frame.gyr_x, frame.gyr_y, frame.gyr_z) == epoch.gyro
