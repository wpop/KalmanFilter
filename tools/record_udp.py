"""Record raw UDP packets with receive timestamps for offline tests.

Each packet is appended to the output file as one line:

    <unix_time_seconds>\t<payload decoded as latin-1 with escapes>\n

Payload bytes are written with backslash escapes (CR, LF, tab, backslash and
non-printable bytes), so one packet always occupies exactly one line and can be
restored byte-for-byte with ``codecs.decode(text, "unicode_escape")``.

Usage:
    python tools/record_udp.py PORT [-o recordings/udp.log] [--host 0.0.0.0]
"""

from __future__ import annotations

import argparse
import socket
import time
from pathlib import Path

MAX_DATAGRAM_SIZE = 65535


def escape_payload(data: bytes) -> str:
    """Make the payload single-line and printable, reversibly."""
    out = []
    for byte in data:
        if byte == 0x5C:
            out.append("\\\\")
        elif byte == 0x0A:
            out.append("\\n")
        elif byte == 0x0D:
            out.append("\\r")
        elif byte == 0x09:
            out.append("\\t")
        elif 0x20 <= byte < 0x7F:
            out.append(chr(byte))
        else:
            out.append(f"\\x{byte:02x}")
    return "".join(out)


def record(host: str, port: int, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, \
            output.open("a", encoding="ascii") as out:
        sock.bind((host, port))
        print(f"Listening on {host}:{port}, writing to {output} (Ctrl+C to stop)")
        count = 0
        try:
            while True:
                data, _addr = sock.recvfrom(MAX_DATAGRAM_SIZE)
                out.write(f"{time.time():.6f}\t{escape_payload(data)}\n")
                out.flush()
                count += 1
        except KeyboardInterrupt:
            print(f"Stopped, {count} packets recorded")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port", type=int, help="UDP port to listen on")
    parser.add_argument("-o", "--output", type=Path, default=Path("recordings/udp.log"),
                        help="file to append packets to")
    parser.add_argument("--host", default="0.0.0.0", help="interface to bind to")
    args = parser.parse_args()
    record(args.host, args.port, args.output)


if __name__ == "__main__":
    main()
