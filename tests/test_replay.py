import io
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pynmea2")

from tools.make_sample_recording import make_packets  # noqa: E402
from tools.record_udp import escape_payload  # noqa: E402
from tools.replay import read_recording, replay, unescape_payload  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SAMPLE = ROOT / "tests" / "data" / "sample_recording.log"


def test_unescape_is_the_inverse_of_escape_for_all_bytes():
    data = bytes(range(256))
    assert unescape_payload(escape_payload(data)) == data


def test_sample_file_matches_the_generator():
    expected = [f"{received:.6f}\t{escape_payload(packet)}\n" for received, packet in make_packets()]
    assert SAMPLE.read_text(encoding="ascii").splitlines(keepends=True) == expected


def test_read_recording_rejects_a_line_without_tab(tmp_path):
    bad = tmp_path / "bad.log"
    bad.write_text("1.0 no tab here\n", encoding="ascii")
    with pytest.raises(ValueError):
        list(read_recording(bad))


def test_replay_of_sample_recording_gives_expected_points():
    out = io.StringIO()
    summary = replay(SAMPLE, out)

    # 11 one-second epochs: the last frame is still incomplete, so 10 points
    assert (summary.packets, summary.frames) == (11, 11)
    assert (summary.accepted, summary.skipped) == (10, 0)
    rows = [line for line in out.getvalue().splitlines() if line[:2] == "12"]
    assert len(rows) == 10
    assert rows[0].startswith("12:00:00") and rows[-1].startswith("12:00:09")
    assert out.getvalue().rstrip().endswith("Packets: 11, frames: 11, points accepted: 10, skipped: 0")


def test_command_line_tool_runs_from_another_directory(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT / "tools" / "replay.py"), str(SAMPLE)],
                            cwd=tmp_path, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert "points accepted: 10" in result.stdout
