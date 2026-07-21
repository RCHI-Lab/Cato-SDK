"""The main :class:`Cato` device class.

Opens the Cato over HID (USB or Bluetooth Classic — pair it at the OS level
first), runs a background reader thread that parses the IMU stream, answers
the mandatory heartbeat, runs sensor fusion, and dispatches data to
callbacks, iterators, and a ``latest()`` accessor.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import replace
from struct import error as struct_error
from typing import Any, Callable, Dict, Iterator, List, Optional

try:
    import hid
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "The 'hidapi' package is required: pip install hidapi"
    ) from e

if not hasattr(hid, "device"):  # pragma: no cover
    raise ImportError(
        "Wrong HID backend: the PyPI package 'hid' shadows 'hidapi' (both "
        "install a module named 'hid'). Run: pip uninstall hid && pip install "
        "--force-reinstall hidapi"
    )

from .exceptions import (
    CatoError,
    CommandError,
    DeviceDisconnectedError,
    DeviceNotFoundError,
)
from .fusion import MadgwickFilter
from .models import ImuSample, PracticeEvent, StatusEvent
from .ota import OtaChannel
from .protocol import (
    HEARTBEAT_REPORT_ID,
    PRACTICE_REPORT_ID,
    PRODUCT_ID,
    SENSOR_REPORT_ID,
    STATUS_REPORT_ID,
    VENDOR_ID,
    parse_practice,
    parse_sensor,
    parse_status,
)

_MAX_FUSION_DT = 0.1  # cap dt across stream gaps, matching the reference demo
_SENTINEL = object()


class Cato:
    """A connection to a Cato device.

    Typical use::

        with Cato() as cato:
            for sample in cato.samples():
                print(sample.acc, sample.gyro, sample.euler())

    Parameters
    ----------
    path:
        Optional HID path (from :meth:`list_devices`) to open a specific
        device. By default every enumerated Cato interface is tried until one
        answers.
    fusion:
        Run the Madgwick filter on the reader thread so samples carry
        ``quaternion`` and ``linear_acc``. Disable for minimum latency if you
        only need raw data.
    queue_size:
        Bound of each :meth:`samples` subscriber queue; the oldest sample is
        dropped when a slow consumer falls behind.
    read_timeout_ms:
        HID read timeout for the reader thread's polling loop.
    """

    def __init__(
        self,
        *,
        path: Optional[bytes] = None,
        fusion: bool = True,
        queue_size: int = 1024,
        read_timeout_ms: int = 100,
    ) -> None:
        self._path = path
        self._fusion_enabled = fusion
        self._queue_size = queue_size
        self._read_timeout_ms = read_timeout_ms

        self._handle: Optional[Any] = None
        self._ota: Optional[OtaChannel] = None
        self._io_lock = threading.Lock()
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._connected = False
        self._streaming = False

        self._filter = MadgwickFilter()
        self._reset_requested = False
        self._last_fusion_ts: Optional[float] = None

        self._latest: Optional[ImuSample] = None
        self._subscribers: List["queue.Queue"] = []
        self._subscribers_lock = threading.Lock()
        self._on_sample: Optional[Callable[[ImuSample], None]] = None
        self._on_status: Optional[Callable[[StatusEvent], None]] = None
        self._on_gesture: Optional[Callable[[PracticeEvent], None]] = None
        self._on_disconnect: Optional[Callable[[Optional[Exception]], None]] = None
        self._info: Optional[Dict[str, Any]] = None

    # ------------------------------------------------------------------ setup

    @staticmethod
    def list_devices() -> List[Dict[str, Any]]:
        """Enumerate connected Cato HID interfaces.

        macOS splits one physical device into one entry per HID collection,
        so several entries for a single Cato are normal.
        """
        return hid.enumerate(VENDOR_ID, PRODUCT_ID)

    def connect(self, *, stream: bool = True) -> None:
        """Open the device and (by default) start the sensor stream.

        Tries each enumerated interface until the ``sensor_stream on``
        command exchange succeeds (with ``stream=False``, the first interface
        that opens is used).
        """
        if self._connected:
            return
        if self._path is not None:
            candidates = [self._path]
        else:
            candidates = [d["path"] for d in self.list_devices()]
        if not candidates:
            raise DeviceNotFoundError(
                "No Cato found. Make sure it is powered on and paired as a "
                "Bluetooth device (or plugged in over USB). On macOS, grant "
                "your terminal Input Monitoring permission in System Settings "
                "> Privacy & Security."
            )

        errors: List[str] = []
        for path in candidates:
            handle = hid.device()
            try:
                handle.open_path(path)
            except (OSError, ValueError) as e:
                errors.append(f"{path!r}: {e}")
                continue

            self._handle = handle
            self._ota = OtaChannel(handle, self._io_lock)
            self._stop.clear()
            self._reader = threading.Thread(
                target=self._reader_loop, name="cato-reader", daemon=True
            )
            self._connected = True
            self._reader.start()

            if not stream:
                return
            try:
                self.start_stream()
                return
            except CommandError as e:
                errors.append(f"{path!r}: {e}")
                self._teardown()

        raise DeviceNotFoundError(
            "Found Cato interface(s) but none accepted commands: "
            + "; ".join(errors)
        )

    def disconnect(self) -> None:
        """Stop streaming (best effort) and close the device."""
        if not self._connected:
            return
        try:
            if self._streaming:
                self.stop_stream()
        except CatoError:
            pass
        self._teardown()

    def _teardown(self, exc: Optional[Exception] = None) -> None:
        self._connected = False
        self._streaming = False
        self._stop.set()
        if self._reader and self._reader is not threading.current_thread():
            self._reader.join(timeout=2.0)
        self._reader = None
        if self._handle is not None:
            try:
                self._handle.close()
            except (OSError, ValueError):
                pass
        self._handle = None
        self._ota = None
        self._last_fusion_ts = None
        self._notify_disconnect(exc)

    def _notify_disconnect(self, exc: Optional[Exception]) -> None:
        with self._subscribers_lock:
            for q in self._subscribers:
                try:
                    q.put_nowait(_SENTINEL)
                except queue.Full:
                    pass
        if self._on_disconnect:
            try:
                self._on_disconnect(exc)
            except Exception:
                pass

    def __enter__(self) -> "Cato":
        self.connect()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.disconnect()

    @property
    def connected(self) -> bool:
        return self._connected

    # -------------------------------------------------------------- streaming

    def start_stream(self) -> None:
        """Ask the device to start streaming IMU reports."""
        self.send_command("sensor_stream on")
        self._streaming = True

    def stop_stream(self) -> None:
        """Ask the device to stop streaming IMU reports."""
        self.send_command("sensor_stream off")
        self._streaming = False

    def samples(self, timeout: Optional[float] = None) -> Iterator[ImuSample]:
        """Iterate over IMU samples as they arrive.

        Blocks waiting for each sample; if ``timeout`` (seconds) elapses with
        no sample, or the device disconnects, iteration ends. Each call gets
        its own bounded queue (oldest samples dropped if you fall behind).
        """
        if not self._connected:
            raise CatoError("Not connected — call connect() first")
        q: "queue.Queue" = queue.Queue(maxsize=self._queue_size)
        with self._subscribers_lock:
            self._subscribers.append(q)
        try:
            while True:
                try:
                    item = q.get(timeout=timeout)
                except queue.Empty:
                    return
                if item is _SENTINEL:
                    return
                yield item
        finally:
            with self._subscribers_lock:
                self._subscribers.remove(q)

    def latest(self) -> Optional[ImuSample]:
        """The most recent sample, without blocking (None before the first one).

        Ideal for render/control loops that run at their own rate.
        """
        return self._latest

    def reset_orientation(self) -> None:
        """Re-zero the fused orientation (like the demo's Reset button)."""
        self._reset_requested = True

    # -------------------------------------------------------------- callbacks

    def on_sample(self, cb: Optional[Callable[[ImuSample], None]]) -> None:
        """Set (or clear) a callback fired for every sample.

        Runs on the reader thread — keep it fast.
        """
        self._on_sample = cb

    def on_status(self, cb: Optional[Callable[[StatusEvent], None]]) -> None:
        """Set (or clear) a callback for status reports (reader thread)."""
        self._on_status = cb

    def on_gesture(self, cb: Optional[Callable[[PracticeEvent], None]]) -> None:
        """Set (or clear) a callback for gesture/practice reports (reader thread)."""
        self._on_gesture = cb

    def on_disconnect(
        self, cb: Optional[Callable[[Optional[Exception]], None]]
    ) -> None:
        """Set (or clear) a callback fired when the connection ends.

        Receives the exception that killed the reader, or ``None`` for a
        clean :meth:`disconnect`.
        """
        self._on_disconnect = cb

    # ----------------------------------------------------------- OTA commands

    def send_command(self, command: str, timeout: float = 3.0) -> Any:
        """Send a text command over the OTA channel and return its response."""
        if self._ota is None:
            raise CatoError("Not connected — call connect() first")
        return self._ota.send_command(command, timeout=timeout)

    def get_info(self) -> Dict[str, Any]:
        """Device info from the ``init`` command (cached)."""
        if self._info is None:
            info = self.send_command("init", timeout=5.0)
            if not isinstance(info, dict):
                raise CommandError(f"Unexpected init response: {info!r}")
            self._info = info
        return self._info

    def get_config(self, path: str = "/global_info") -> Any:
        """Fetch a config subtree, e.g. ``/global_info`` or ``/profiles/0``."""
        return self.send_command(f"config get {path}")

    def get_profiles(self, max_profiles: int = 20) -> List[Any]:
        """Fetch all configured profiles."""
        profiles: List[Any] = []
        for i in range(max_profiles):
            try:
                profile = self.get_config(f"/profiles/{i}")
            except CommandError:
                break
            if profile is None:
                break
            profiles.append(profile)
        return profiles

    # ------------------------------------------------------------ reader loop

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._handle.read(64, timeout_ms=self._read_timeout_ms)
            except (OSError, ValueError) as e:
                if not self._stop.is_set():
                    self._connected = False
                    self._streaming = False
                    self._notify_disconnect(DeviceDisconnectedError(str(e)))
                return
            if not data:
                continue
            report_id = data[0]
            payload = bytes(data[1:])
            try:
                if report_id == SENSOR_REPORT_ID:
                    self._handle_sensor(payload)
                elif report_id == STATUS_REPORT_ID:
                    self._handle_status(payload)
                elif report_id == PRACTICE_REPORT_ID:
                    self._handle_practice(payload)
            except (ValueError, IndexError, struct_error):
                continue  # malformed report — skip

    def _handle_sensor(self, payload: bytes) -> None:
        now = time.monotonic()
        sample = parse_sensor(payload, now)

        if self._fusion_enabled:
            if self._reset_requested:
                self._filter.reset()
                self._last_fusion_ts = None
                self._reset_requested = False
            if self._last_fusion_ts is not None:
                dt = min(now - self._last_fusion_ts, _MAX_FUSION_DT)
                q = self._filter.update(sample.gyro, sample.acc, dt)
                lin = self._filter.linear_acceleration(sample.acc)
                sample = replace(sample, quaternion=q, linear_acc=lin)
            else:
                sample = replace(
                    sample,
                    quaternion=self._filter.q,
                    linear_acc=(0.0, 0.0, 0.0),
                )
            self._last_fusion_ts = now

        self._latest = sample
        with self._subscribers_lock:
            for q_ in self._subscribers:
                try:
                    q_.put_nowait(sample)
                except queue.Full:
                    try:
                        q_.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        q_.put_nowait(sample)
                    except queue.Full:
                        pass
        if self._on_sample:
            self._on_sample(sample)

    def _handle_status(self, payload: bytes) -> None:
        # Heartbeat first — the device drops the connection without it.
        with self._io_lock:
            try:
                self._handle.write(bytes([HEARTBEAT_REPORT_ID, 0x00]))
            except (OSError, ValueError):
                pass
        if self._on_status:
            self._on_status(parse_status(payload))

    def _handle_practice(self, payload: bytes) -> None:
        if self._on_gesture:
            self._on_gesture(parse_practice(payload))
