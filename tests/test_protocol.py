import struct

import pytest

from cato.protocol import (
    OTA_REPORT_ID,
    OTA_WRITE_MAX_DATA,
    REPORT_LEN,
    OtaMsgType,
    build_ota_chunks,
    parse_practice,
    parse_sensor,
    parse_status,
)


def make_sensor_payload(ts, acc_y, acc_z, acc_x, gyro_y, gyro_z, gyro_x):
    return struct.pack("<I6f", ts, acc_y, acc_z, acc_x, gyro_y, gyro_z, gyro_x)


class TestParseSensor:
    def test_axis_remap_and_negation(self):
        payload = make_sensor_payload(1234, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        s = parse_sensor(payload, host_ts=42.0)
        assert s.device_ts == 1234
        assert s.host_ts == 42.0
        # wire order is (acc_y, acc_z, acc_x); sample.acc is (x, y, z)
        assert s.acc == (3.0, 1.0, 2.0)
        # gyro y and z are negated
        assert s.gyro == (6.0, -4.0, -5.0)

    def test_raw_is_untouched_wire_order(self):
        payload = make_sensor_payload(7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        s = parse_sensor(payload, host_ts=0.0)
        assert s.raw == (7, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def test_no_fusion_fields_by_default(self):
        s = parse_sensor(make_sensor_payload(0, 0, 0, 0, 0, 0, 0), 0.0)
        assert s.quaternion is None
        assert s.linear_acc is None
        with pytest.raises(ValueError):
            s.euler()

    def test_short_payload_raises(self):
        with pytest.raises(struct.error):
            parse_sensor(b"\x00" * 10, 0.0)


class TestParseStatus:
    def test_fields_and_bits(self):
        s = parse_status(bytes([3, 5, 0x80 | 0x08 | 0x01, 2]))
        assert s.gesture == 3
        assert s.action == 5
        assert s.con_idx == 2
        assert s.hb_lost and s.scrolling and s.dwell_click
        assert not (
            s.deep_sleep
            or s.unsaved_changes
            or s.pointer_sleep
            or s.practice
            or s.hold_til_idle
        )


class TestParsePractice:
    def test_entries_parsed_and_zero_confidence_dropped(self):
        # entry: low byte = gesture index, bits 8-14 = confidence, bit 15 = threshold
        e1 = 0x8000 | (0x50 << 8) | 2  # gesture 2, confidence 0x50, crossed
        e2 = (0x10 << 8) | 4  # gesture 4, confidence 0x10, not crossed
        e3 = 5  # zero confidence -> dropped
        payload = struct.pack("<IB3H", 999, 2, e1, e2, e3)
        p = parse_practice(payload)
        assert p.device_ts == 999
        assert p.latched == 2
        assert len(p.gestures) == 2
        assert p.gestures[0].index == 2
        assert p.gestures[0].confidence == 0x50
        assert p.gestures[0].threshold_crossed is True
        assert p.gestures[1].index == 4
        assert p.gestures[1].confidence == 0x10
        assert p.gestures[1].threshold_crossed is False


class TestBuildOtaChunks:
    def test_short_command_single_chunk(self):
        chunks = build_ota_chunks("sensor_stream on")
        assert len(chunks) == 1
        report = chunks[0]
        assert len(report) == 1 + REPORT_LEN
        assert report[0] == OTA_REPORT_ID
        assert report[1] == 0x00
        assert report[2] == 0xFF
        assert report[3] == OtaMsgType.FIRST
        length = report[4] | (report[5] << 8)
        assert length == len(b"sensor_stream on")
        assert report[6 : 6 + length] == b"sensor_stream on"
        assert all(b == 0 for b in report[6 + length :])

    def test_long_command_chunked_with_remaining_lengths(self):
        command = "x" * (OTA_WRITE_MAX_DATA + 50)
        chunks = build_ota_chunks(command)
        assert len(chunks) == 2
        first, second = chunks
        assert first[3] == OtaMsgType.FIRST
        assert second[3] == OtaMsgType.MID
        assert (first[4] | first[5] << 8) == len(command)
        assert (second[4] | second[5] << 8) == 50
        data = first[6 : 6 + OTA_WRITE_MAX_DATA] + second[6 : 6 + 50]
        assert data == command.encode()
