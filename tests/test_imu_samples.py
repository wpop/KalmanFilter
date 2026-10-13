"""Tests that every IMU reading is stored in the list of its own frame."""

import logging
import threading
from typing import Sequence

import pytest

pytest.importorskip("pynmea2")

from Sensor_Data import ImuSample  # noqa: E402
from tests.test_parser_frames import (EPOCH_1, EPOCH_2, _SequenceSocket, _sentences,  # noqa: E402,F401
                                      feed, nmea, parser)


def imu_packet(packet_time: str, sentence: str, groups: Sequence[Sequence[str]]) -> bytes:
    """One NMEA sentence followed by any number of IMU groups: (kind, x, y, z)."""
    fields = [packet_time, "1", nmea(sentence), "0", "0"]
    for group in groups:
        fields += list(group)
    return (",".join(fields) + "\n").encode("utf-8")


def test_multiple_samples_of_one_epoch_are_kept_in_order(parser, caplog):
    rmc = _sentences(EPOCH_1)["RMC"]
    packets = [
        imu_packet("100.0", rmc, [("3", "0.1", "0.2", "9.8"), ("4", "1", "2", "3"), ("5", "10", "20", "30")]),
        imu_packet("100.1", "GPGSA,A,3,01,,,,,,,,,,,,1.8,1.2,1.4",
                   [("3", "0.3", "0.4", "9.7"), ("3", "0.5", "0.6", "9.6"), ("4", "4", "5", "6")]),
    ]
    feed(parser, packets, caplog)

    assert len(parser.frames) == 1
    frame = parser.frames[0]
    assert frame.accel_samples == [ImuSample("100.0", 0.1, 0.2, 9.8),
                                   ImuSample("100.1", 0.3, 0.4, 9.7),
                                   ImuSample("100.1", 0.5, 0.6, 9.6)]
    assert frame.gyro_samples == [ImuSample("100.0", 1.0, 2.0, 3.0),
                                  ImuSample("100.1", 4.0, 5.0, 6.0)]
    assert frame.mag_samples == [ImuSample("100.0", 10.0, 20.0, 30.0)]
    # the old fields still hold the last reading, as strings
    assert (frame.acceleration_x, frame.acceleration_y, frame.acceleration_z) == ("0.5", "0.6", "9.6")


def test_samples_are_not_mixed_between_epochs(parser, caplog):
    packets = [
        imu_packet("100.0", _sentences(EPOCH_1)["RMC"], [("3", "1", "1", "1"), ("3", "2", "2", "2")]),
        imu_packet("100.5", _sentences(EPOCH_2)["RMC"], [("3", "3", "3", "3"), ("4", "7", "7", "7")]),
        imu_packet("100.6", "GPGSA,A,3,01,,,,,,,,,,,,1.8,1.2,1.4", [("3", "4", "4", "4")]),
    ]
    feed(parser, packets, caplog)

    newest, oldest = parser.frames[0], parser.frames[1]
    assert oldest.accel_samples == [ImuSample("100.0", 1.0, 1.0, 1.0), ImuSample("100.0", 2.0, 2.0, 2.0)]
    assert oldest.gyro_samples == []
    assert newest.accel_samples == [ImuSample("100.5", 3.0, 3.0, 3.0), ImuSample("100.6", 4.0, 4.0, 4.0)]
    assert newest.gyro_samples == [ImuSample("100.5", 7.0, 7.0, 7.0)]


def test_non_numeric_sample_is_logged_and_skipped(parser, caplog):
    rmc = _sentences(EPOCH_1)["RMC"]
    packet = imu_packet("100.0", rmc, [("3", "abc", "0.2", "9.8"), ("3", "0.3", "0.4", "9.7"),
                                       ("4", "1", "2", "3")])
    # feed() only captures ERROR records, so run the parser directly to see the warning
    parser.connection_mgr.m_socket = _SequenceSocket([packet])
    with caplog.at_level(logging.WARNING):
        parser.parser_gps(threading.Lock())
    assert not [r for r in caplog.records if r.levelno >= logging.ERROR]

    frame = parser.frames[0]
    assert frame.accel_samples == [ImuSample("100.0", 0.3, 0.4, 9.7)]
    assert frame.gyro_samples == [ImuSample("100.0", 1.0, 2.0, 3.0)]
    warnings = [r.getMessage() for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1 and "accelerometer" in warnings[0] and "abc" in warnings[0]
