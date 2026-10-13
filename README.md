# KalmanFilter

A Kalman filter for a rover that uses GPS data (and, in the future, IMU data) streamed from
phones over UDP. The planned setup has two phones: a **rover** that moves and a stationary
**base station**.

## Status

Be aware that this is work in progress. Everything below was checked only on synthetic data.

**Works (covered by tests, `python -m pytest`):**

- Linear Kalman filter with swappable motion and measurement models (`kalman_core.py`).
- Latitude/longitude to local east/north meters and back (`geo_utils.py`).
- GPS-only position tracker (`tracker.py`).
- Parser that assembles UDP packets into per-second frames (`GPS_Parser.py`).
- Adapter from frames to GPS fixes (`frame_adapter.py`) and a pipeline that feeds new frames
  to the tracker (`position_pipeline.py`).
- Tools to record UDP traffic and replay a recording through the whole chain (`tools/`).

**Does not work or is not verified:**

- `KalmanFilter.py` (the `Rover` / `Stationary` classes started with `multiprocessing`) does
  not work. It creates `Rover` and `Stationary` at import time, which opens UDP ports 5556
  and 5555, and `Rover.run_kalman` calls `parser.get_data()` without the required lock
  argument. It does not use the Kalman filter at all. It is left as is for now.
- The filter does not use IMU data yet. The parser stores every accelerometer, gyroscope and
  magnetometer reading of a frame (`accel_samples`, `gyro_samples`, `mag_samples`), but nothing
  consumes them.
- The parser expects the packet format of one particular Android app and was tested only on
  artificial packets built from that format as understood from the code. The meaning of the
  first field of a packet is unknown.
- The order of NMEA sentences from a real phone has not been checked. The parser assumes that
  a frame starts with the first RMC or GGA sentence whose timestamp differs from the current
  frame, and that GSA, GSV and IMU data follow it. Data that arrives before the first RMC or GGA
  is discarded.
- `geohash2` is only needed for a debug print in `KalmanFilter.py`; it is imported lazily there.

## Data flow

```
UDP packets -> Parser -> frames -> FrameToGpsFixAdapter -> PositionTracker (Kalman filter)
                                                              -> position in meters (east, north)
```

`PositionPipeline.poll()` connects the stages: it takes completed frames from the parser,
restores chronological order, skips frames it has already processed and returns a
`PolledPoint` (fix, whether the tracker applied it, filtered position) for each new fix.
The newest frame is never returned because it may still be incomplete.

## Modules

| Module | Responsibility |
|---|---|
| `kalman_core.py` | `KalmanFilter` (predict/update, Joseph form), `MotionModel` / `MeasurementModel` abstractions, `ConstantAccelerationModel2D`, `PositionMeasurement`, `PositionVelocityMeasurement`. State: `[east, north, v_east, v_north]`. |
| `geo_utils.py` | `LocalTangentPlane`: WGS84-based conversion between lat/lon and local meters. |
| `tracker.py` | `GpsFix` and `PositionTracker`: the first fix sets the origin; each next fix runs predict and update; fixes with `dt <= 0` are skipped. GPS only. |
| `GPS_Parser.py` | `Parser`: reads UDP packets (NMEA with `;` separators plus IMU groups) and builds frames. |
| `Sensor_Data.py` | `Sensors` (one frame) and `ImuSample`. |
| `connection.py` | UDP socket wrapper used by the parser. |
| `frame_adapter.py` | `FrameToGpsFixAdapter`: frame to `GpsFix`; returns `None` if time, date or coordinates are missing; falls back to a default HDOP when it is missing or invalid. |
| `position_pipeline.py` | `PositionPipeline`: feeds new frames to the tracker. No sockets or threads. |
| `KalmanFilter.py` | Legacy `Rover` / `Stationary` runner, see Status. |
| `tools/record_udp.py` | Records UDP packets with receive times to a file. |
| `tools/replay.py` | Replays a recording through parser, pipeline and tracker and prints a table. |
| `tools/make_sample_recording.py` | Generates the synthetic recording in `tests/data/`. |

## Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt -r requirements-dev.txt
```

Run the tests:

```bash
python -m pytest
```

The standalone filter demo on a simulated track (`python kalman_core.py`) prints:

```
Raw GPS RMSE : 3.82 m
Filter RMSE  : 2.16 m
```

## Demo: replaying a recording

```bash
python tools/replay.py tests/data/sample_recording.log
```

The sample is a synthetic 10-second track (about 2 m/s east and 1 m/s north, 1 m GPS noise).
The fifth epoch has no GSA sentence, so its HDOP is missing and the fallback value 5.0 is used.
Positions are meters from the first fix.

```
time (UTC)     latitude    longitude   hdop    east, m   north, m  status
12:00:00      55.759988    37.569987    1.0       0.00       0.00  ok
12:00:01      55.760013    37.570028    1.0       2.02       2.17  ok
12:00:02      55.760019    37.570082    1.0       5.36       3.52  ok
12:00:03      55.760020    37.570087    1.0       6.79       4.00  ok
12:00:04      55.760051    37.570139    1.0       9.27       6.18  ok
12:00:05      55.760034    37.570164    5.0      11.46       7.51  ok
12:00:06      55.760068    37.570176    1.0      12.46       8.92  ok
12:00:07      55.760047    37.570226    1.0      14.69       8.36  ok
12:00:08      55.760061    37.570253    1.0      16.69       8.74  ok
12:00:09      55.760076    37.570277    1.0      18.45       9.65  ok

Packets: 11, frames: 11, points accepted: 10, skipped: 0
```

Eleven packets give ten points because the last frame is still incomplete. Add `-v` to see the
parser's debug logging. To regenerate the sample: `python tools/make_sample_recording.py`.

## Recording real traffic

```bash
python tools/record_udp.py <port> -o recordings/<name>.log
```

Stop it with Ctrl+C. Each packet is appended as one line: the receive time, a tab and the packet
with escaped bytes. The `recordings/` directory is ignored by git. Replay the recording the same
way as the sample:

```bash
python tools/replay.py recordings/<name>.log
```

## Known limitations and plans

- The first fix is used as the origin of the local plane and as the initial position, so its
  noise becomes a constant offset of the whole track.
- Several values are assumptions that must be calibrated on real data: `DEFAULT_UERE_M = 3.0`
  in `PositionMeasurement` (position std is about HDOP times UERE), the fallback HDOP of 5.0 in
  `FrameToGpsFixAdapter`, and the process noise and initial velocity uncertainty in
  `PositionTracker` (`DEFAULT_SIGMA_ACC = 0.5`, `INITIAL_SIGMA_VEL = 5.0`).
- Real recordings from a phone are needed to check the packet format and the sentence order.
- IMU: rotate accelerometer readings into the east/north axes and use them as the filter control
  input.
- Use the base station (for example, to correct the rover's position).
- Replace the legacy `multiprocessing` runner in `KalmanFilter.py`.

## License

No license has been chosen yet.
