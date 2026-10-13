import logging
import threading

import pytest

pytest.importorskip("pynmea2")

from GPS_Parser import Parser  # noqa: E402


class _RaisingSocket:
    """Socket stub whose recvfrom raises the given exception."""

    def __init__(self, exc: BaseException) -> None:
        self._exc = exc

    def recvfrom(self, _size: int):
        raise self._exc


@pytest.fixture
def parser():
    # Port 0 lets the OS pick a free port, so no real port is needed
    parser = Parser(0)
    real_socket = parser.connection_mgr.m_socket  # tests may replace it with a stub
    yield parser
    real_socket.close()


def test_get_data_on_fresh_parser_returns_empty_deque(parser):
    data = parser.get_data(threading.Lock())
    assert len(data) == 0


def test_keyboard_interrupt_is_propagated(parser):
    parser.connection_mgr.m_socket = _RaisingSocket(KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        parser.parser_gps(threading.Lock())


def test_regular_exception_is_logged_and_swallowed(parser, caplog):
    parser.connection_mgr.m_socket = _RaisingSocket(OSError("socket broke"))
    with caplog.at_level(logging.ERROR):
        parser.parser_gps(threading.Lock())
    errors = [r for r in caplog.records if r.levelno == logging.ERROR]
    assert len(errors) == 1
    assert errors[0].exc_info[0] is OSError
