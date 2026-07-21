# Cato-SDK

Python SDK for realtime 3D IMU data from the [Auli.tech Cato](https://auli.tech/).
The Cato's standard HID mouse output is only 2D; this SDK exposes the full 3-axis
accelerometer + gyroscope stream, fused orientation (quaternion / euler), and
linear acceleration for robotics and research applications.

The protocol is a Python port of Auli's official
[imu_visualizer](https://github.com/aulitech/imu_visualizer) web demo. The
device communicates over plain HID (USB or Bluetooth Classic), **not** BLE
GATT — pair it at the OS level like any Bluetooth device and the SDK will
find it.

## Install

```bash
pip install -e ".[examples]"      # from a clone; [examples] adds matplotlib + websockets
```

> **Note:** the SDK depends on the PyPI package `hidapi` (imported as `hid`).
> Do **not** also install the PyPI package named `hid` — both provide a module
> called `hid` and shadow each other. If things break:
> `pip uninstall hid && pip install --force-reinstall hidapi`.

### macOS setup

1. Pair the Cato in **System Settings → Bluetooth** (or plug it in over USB).
2. Grant your terminal app **Input Monitoring** permission in
   **System Settings → Privacy & Security → Input Monitoring** — required for
   HID access. Restart the terminal after granting.

### Linux setup

Add a udev rule so you can open the device without root:

```
# /etc/udev/rules.d/99-cato.rules
KERNEL=="hidraw*", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="52dd", MODE="0666"
```

## Quickstart

```python
from cato import Cato

with Cato() as cato:                      # finds the device, starts the stream
    for sample in cato.samples():
        print(sample.acc)                 # (x, y, z) m/s²
        print(sample.gyro)                # (x, y, z) deg/s
        print(sample.quaternion)          # (w, x, y, z) fused orientation
        print(sample.linear_acc)          # gravity-removed, world frame
        print(sample.euler())             # (roll, pitch, yaw) degrees
```

Three ways to consume data, all driven by one background reader thread:

```python
# 1. Iterator — natural for logging/scripts
for sample in cato.samples(timeout=1.0):
    ...

# 2. Latest value — natural for render/control loops at their own rate
sample = cato.latest()

# 3. Callbacks — fired on the reader thread, keep them fast
cato.on_sample(lambda s: ...)
cato.on_status(lambda st: ...)        # device status changes
cato.on_gesture(lambda g: ...)        # gesture inference reports
cato.on_disconnect(lambda exc: ...)
```

Other useful bits:

```python
Cato.list_devices()                   # enumerate connected Catos
cato.reset_orientation()              # re-zero the fused orientation
cato.send_command("sensor_stream on") # raw text-command channel
cato.get_info()                       # device info ("init" command)
cato.get_config("/global_info")       # config subtrees
cato.get_profiles()
Cato(fusion=False)                    # raw-only, no Madgwick filter
```

## Examples

All in [`examples/`](examples/) — run with the device paired and powered on:

| Example | What it does |
|---|---|
| `01_print_stream.py` | Print live acc / gyro / euler at ~10 Hz |
| `02_csv_logger.py` | Log raw + fused samples to CSV until Ctrl-C |
| `03_live_plot.py` | Realtime matplotlib traces of acc & gyro |
| `04_gesture_monitor.py` | Print status changes and gesture inferences |
| `visualizer/serve.py` | **3D orientation visualizer in your browser** (like Auli's demo): Python streams fused data over a websocket to a three.js page |

```bash
python examples/visualizer/serve.py   # then open http://localhost:8000 (opens automatically)
```

## Data conventions

- **Frame:** samples are given in the same Y-up reference frame as Auli's
  imu_visualizer (device axes remapped to x/y/z; gyro y/z negated), because the
  sensor-fusion constants were tuned in that frame. At rest, `acc ≈ (0, -9.81, 0)`.
- **Raw wire values:** `sample.raw` keeps the untouched on-wire tuple
  `(ts, acc_y, acc_z, acc_x, gyro_y, gyro_z, gyro_x)`.
- **Units:** already physical on the wire — acc in m/s², gyro in deg/s.
- **Orientation:** IMU-only Madgwick filter (no magnetometer), so yaw is
  relative, not absolute heading. Quaternions are `(w, x, y, z)`.
- **Sample rate:** whatever the firmware streams; each sample carries the
  device's `uint32` timestamp (`device_ts`) and a host `time.monotonic()`
  stamp (`host_ts`).

## Protocol notes

For the curious (reverse-engineered from the official web demo):

- HID device, VID `0x1915` / PID `0x52DD`.
- Report 7 (input): IMU stream — `<I6f` little-endian, timestamp + 6 floats.
- Report 6 (feature): text-command channel ("OTA") with chunked framing —
  commands like `sensor_stream on`, `init`, `config get /global_info`.
- Report 9 (input): status; **must** be answered with a heartbeat (report 10,
  one zero byte) or the device drops the link — the SDK does this automatically.
- Report 8 (input): gesture-inference results.

## Development

```bash
pip install -e ".[dev]"
pytest                                # protocol/fusion/OTA tests, no hardware needed
```

## License

MIT
