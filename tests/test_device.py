"""End-to-end Cato tests against a fake hidapi backend (no hardware)."""

import queue
import struct
import time

import pytest

import cato.device as device_mod
from cato import Cato, DeviceNotFoundError
from cato.protocol import (
    HEARTBEAT_REPORT_ID,
    OTA_REPORT_ID,
    SENSOR_REPORT_ID,
    STATUS_REPORT_ID,
    OtaMsgType,
)


def sensor_report(ts=0, acc=(0.0, -9.81, 0.0), gyro=(0.0, 0.0, 0.0)):
    """Input report bytes as read() returns them (report ID first).

    acc/gyro given in the reference frame; converted back to wire order.
    """
    wire = struct.pack(
        "<I6f", ts, acc[1], acc[2], acc[0], -gyro[1], -gyro[2], gyro[0]
    )
    return list(bytes([SENSOR_REPORT_ID]) + wire)


def status_report(bits=0x01):
    return [STATUS_REPORT_ID, 0, 0, bits, 0]


class FakeHandle:
    """Reports are pushed in with feed() after the test has set up consumers,
    so tests are deterministic regardless of reader-thread timing."""

    def __init__(self):
        self.incoming = queue.Queue()
        self.written = []
        self.feature_sent = []
        self.closed = False

    def feed(self, *reports):
        for r in reports:
            self.incoming.put(r)

    def open_path(self, path):
        pass

    def read(self, size, timeout_ms=0):
        try:
            return self.incoming.get(timeout=timeout_ms / 1000)
        except queue.Empty:
            return []

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)

    def send_feature_report(self, data):
        self.feature_sent.append(bytes(data))
        return len(data)

    def get_feature_report(self, report_id, length):
        # Always ack the last command
        return [OTA_REPORT_ID, 0x00, 0xFF, OtaMsgType.ACK_NRSP, 0, 0]

    def close(self):
        self.closed = True


@pytest.fixture
def fake_hid(monkeypatch):
    state = {"handle": None, "devices": [{"path": b"fake-path"}]}

    class FakeHidModule:
        @staticmethod
        def enumerate(vid, pid):
            return state["devices"]

        @staticmethod
        def device():
            return state["handle"]

    monkeypatch.setattr(device_mod, "hid", FakeHidModule)
    return state


def wait_for(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert predicate()


def test_no_device_raises(fake_hid):
    fake_hid["devices"] = []
    with pytest.raises(DeviceNotFoundError):
        Cato().connect()


def test_connect_stream_and_disconnect(fake_hid):
    handle = FakeHandle()
    fake_hid["handle"] = handle

    with Cato() as cato:
        assert cato.connected
        # sensor_stream on was sent over the OTA channel
        assert any(b"sensor_stream on" in r for r in handle.feature_sent)
        handle.feed(*[sensor_report(ts=i) for i in range(5)])
        samples = []
        for s in cato.samples(timeout=1.0):
            samples.append(s)
            if len(samples) == 5:
                break
    assert not cato.connected
    assert handle.closed
    assert any(b"sensor_stream off" in r for r in handle.feature_sent)

    assert [s.device_ts for s in samples] == [0, 1, 2, 3, 4]
    assert samples[0].acc == pytest.approx((0.0, -9.81, 0.0))
    assert samples[-1].quaternion is not None
    assert abs(samples[-1].quaternion[0]) > 0.99  # still ~identity at rest


def test_status_report_triggers_heartbeat(fake_hid):
    handle = FakeHandle()
    fake_hid["handle"] = handle

    events = []
    with Cato() as cato:
        cato.on_status(events.append)
        handle.feed(status_report())
        wait_for(lambda: handle.written)
    assert handle.written == [bytes([HEARTBEAT_REPORT_ID, 0x00])]
    assert len(events) == 1
    assert events[0].dwell_click is True


def test_latest_and_callback(fake_hid):
    handle = FakeHandle()
    fake_hid["handle"] = handle

    seen = []
    with Cato() as cato:
        cato.on_sample(seen.append)
        handle.feed(sensor_report(ts=42))
        wait_for(lambda: cato.latest() is not None)
        assert cato.latest().device_ts == 42
    assert len(seen) == 1


def test_fusion_disabled_gives_raw_only(fake_hid):
    handle = FakeHandle()
    fake_hid["handle"] = handle

    with Cato(fusion=False) as cato:
        handle.feed(sensor_report())
        s = next(iter(cato.samples(timeout=1.0)))
    assert s.quaternion is None
    assert s.linear_acc is None
