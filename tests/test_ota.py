import json
import threading

import pytest

from cato.exceptions import CommandError, CommandTimeoutError
from cato.ota import OtaChannel
from cato.protocol import OTA_REPORT_ID, OtaMsgType


def make_response(msg_type, payload=b"", length=None):
    """Build a fake get_feature_report() return value (6-byte header + data)."""
    if length is None:
        length = len(payload)
    return list(
        bytes([OTA_REPORT_ID, 0x00, 0xFF, msg_type, length & 0xFF, length >> 8])
        + payload
    )


class FakeHandle:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []

    def send_feature_report(self, data):
        self.sent.append(bytes(data))
        return len(data)

    def get_feature_report(self, report_id, length):
        assert report_id == OTA_REPORT_ID
        if not self.responses:
            return []
        return self.responses.pop(0)


def make_channel(responses):
    handle = FakeHandle(responses)
    return OtaChannel(handle, threading.Lock()), handle


def test_json_response_single_packet():
    body = json.dumps({"version": "1.2.3"}).encode()
    ch, handle = make_channel([make_response(OtaMsgType.FIRST, body)])
    assert ch.send_command("init") == {"version": "1.2.3"}
    assert len(handle.sent) == 1
    assert handle.sent[0][0] == OTA_REPORT_ID


def test_multi_packet_first_mid_reassembly():
    part1, part2 = b'{"a": 1, "b"', b': 2}'
    total = len(part1) + len(part2)
    ch, _ = make_channel(
        [
            make_response(OtaMsgType.FIRST, part1, length=total),
            make_response(OtaMsgType.MID, part2, length=len(part2)),
        ]
    )
    assert ch.send_command("config get /global_info") == {"a": 1, "b": 2}


def test_ack_nrsp_returns_none():
    ch, _ = make_channel([make_response(OtaMsgType.ACK_NRSP)])
    assert ch.send_command("sensor_stream on") is None


def test_ack_rsp_waits_for_following_packet():
    ch, _ = make_channel(
        [
            make_response(OtaMsgType.ACK_RSP),
            make_response(OtaMsgType.FIRST, b"ok"),
        ]
    )
    assert ch.send_command("cmd") == "ok"


def test_result_replaces_data():
    ch, _ = make_channel([make_response(OtaMsgType.RESULT, b'"done"')])
    assert ch.send_command("cmd") == "done"


def test_nack_raises_command_error():
    ch, _ = make_channel([make_response(OtaMsgType.NACK, b"bad command")])
    with pytest.raises(CommandError, match="bad command"):
        ch.send_command("nonsense")


def test_empty_raises_command_error():
    ch, _ = make_channel([make_response(OtaMsgType.EMPTY)])
    with pytest.raises(CommandError):
        ch.send_command("cmd")


def test_no_response_times_out():
    ch, _ = make_channel([])
    with pytest.raises(CommandTimeoutError):
        ch.send_command("cmd", timeout=0.1)


def test_non_json_response_returned_as_string():
    ch, _ = make_channel([make_response(OtaMsgType.FIRST, b"plain text")])
    assert ch.send_command("cmd") == "plain text"


def test_long_command_sends_multiple_chunks():
    ch, handle = make_channel([make_response(OtaMsgType.ACK_NRSP)])
    ch.send_command("x" * 300)
    assert len(handle.sent) == 2
