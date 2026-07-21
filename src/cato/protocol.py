"""Cato HID protocol: constants, report parsing, and OTA command framing.

Everything in this module is pure (no I/O) so it can be unit-tested without
hardware. Protocol reverse-engineered from Auli's imu_visualizer
(``src/useWebHID.js`` and ``src/ota.jsx``).

Sensor input report (ID 7), little-endian::

    bytes  0- 3: uint32  timestamp
    bytes  4- 7: float32 acc.y
    bytes  8-11: float32 acc.z
    bytes 12-15: float32 acc.x
    bytes 16-19: float32 gyro.y  (negated for the reference frame)
    bytes 20-23: float32 gyro.z  (negated for the reference frame)
    bytes 24-27: float32 gyro.x

Units are already physical on the wire: acc in m/s², gyro in deg/s.
"""

from __future__ import annotations

import struct
from enum import IntEnum
from typing import List

from .models import GestureResult, ImuSample, PracticeEvent, StatusEvent

VENDOR_ID = 0x1915  # Nordic Semiconductor
PRODUCT_ID = 0x52DD

OTA_REPORT_ID = 6  # feature report: text-command channel
SENSOR_REPORT_ID = 7  # input report: IMU stream
PRACTICE_REPORT_ID = 8  # input report: gesture inference
STATUS_REPORT_ID = 9  # input report: device status (requires heartbeat reply)
HEARTBEAT_REPORT_ID = 10  # output report: heartbeat ack

REPORT_LEN = 244  # OTA feature-report payload length (excluding report ID)
OTA_WRITE_HEADER_LEN = 5  # [0x00, 0xFF, msgType, lenLSB, lenMSB]
OTA_WRITE_MAX_DATA = REPORT_LEN - OTA_WRITE_HEADER_LEN
OTA_READ_HEADER_LEN = 6  # [reportId, 0x00, 0xFF, msgType, lenLSB, lenMSB]

SENSOR_STRUCT = struct.Struct("<I6f")


class OtaMsgType(IntEnum):
    FIRST = 0
    MID = 1
    ACK_NRSP = 2
    ACK_RSP = 3
    NACK = 4
    EMPTY = 5
    RESULT = 6


def parse_sensor(payload: bytes, host_ts: float) -> ImuSample:
    """Parse a sensor input report payload (report ID byte already stripped)."""
    ts, acc_y, acc_z, acc_x, gyro_y, gyro_z, gyro_x = SENSOR_STRUCT.unpack_from(
        payload
    )
    return ImuSample(
        device_ts=ts,
        host_ts=host_ts,
        acc=(acc_x, acc_y, acc_z),
        gyro=(gyro_x, -gyro_y, -gyro_z),
        raw=(ts, acc_y, acc_z, acc_x, gyro_y, gyro_z, gyro_x),
    )


def parse_status(payload: bytes) -> StatusEvent:
    """Parse a status input report payload (report ID byte already stripped)."""
    gesture, action, status, con_idx = payload[0], payload[1], payload[2], payload[3]
    return StatusEvent(
        gesture=gesture,
        action=action,
        con_idx=con_idx,
        hb_lost=bool(status & 0x80),
        deep_sleep=bool(status & 0x40),
        unsaved_changes=bool(status & 0x20),
        pointer_sleep=bool(status & 0x10),
        scrolling=bool(status & 0x08),
        practice=bool(status & 0x04),
        hold_til_idle=bool(status & 0x02),
        dwell_click=bool(status & 0x01),
    )


def parse_practice(payload: bytes) -> PracticeEvent:
    """Parse a practice/gesture input report payload.

    Entries with zero confidence are dropped, matching the reference app.
    """
    (ts,) = struct.unpack_from("<I", payload, 0)
    latched = payload[4]
    gestures = []
    body = payload[5:]
    for (value,) in struct.iter_unpack("<H", body[: len(body) - len(body) % 2]):
        if (value & 0x7F00) == 0:  # zero confidence
            continue
        gestures.append(
            GestureResult(
                index=value & 0x00FF,
                confidence=(value & 0x7F00) >> 8,
                threshold_crossed=bool(value & 0x8000),
            )
        )
    return PracticeEvent(device_ts=ts, latched=latched, gestures=tuple(gestures))


def build_ota_chunks(command: str) -> List[bytes]:
    """Frame a text command into OTA feature reports.

    Each returned chunk is ``1 + REPORT_LEN`` bytes: the leading report-ID
    byte (as hidapi's ``send_feature_report`` expects) followed by the 5-byte
    header and payload, zero-padded. Commands longer than one report are split
    into a FIRST chunk followed by MID chunks; the length field carries the
    total *remaining* payload length at each chunk.
    """
    payload = command.encode("utf-8")
    chunks: List[bytes] = []
    offset = 0
    while offset < len(payload) or not chunks:
        remaining = len(payload) - offset
        chunk_len = min(OTA_WRITE_MAX_DATA, remaining)
        msg_type = OtaMsgType.FIRST if not chunks else OtaMsgType.MID
        report = bytearray(1 + REPORT_LEN)
        report[0] = OTA_REPORT_ID
        report[1:6] = bytes(
            [0x00, 0xFF, msg_type, remaining & 0xFF, (remaining >> 8) & 0xFF]
        )
        report[6 : 6 + chunk_len] = payload[offset : offset + chunk_len]
        chunks.append(bytes(report))
        offset += chunk_len
    return chunks
