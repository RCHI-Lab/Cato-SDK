"""OTA text-command channel over HID feature reports (report ID 6).

Commands are ASCII strings (e.g. ``"sensor_stream on"``); responses are
reassembled from one or more feature reports and JSON-decoded when possible.
The device handles one exchange at a time, so exchanges are serialized with a
lock and are safe to run concurrently with the streaming reader thread.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

from .exceptions import CommandError, CommandTimeoutError
from .protocol import (
    OTA_READ_HEADER_LEN,
    OTA_REPORT_ID,
    REPORT_LEN,
    OtaMsgType,
    build_ota_chunks,
)

_POLL_INTERVAL = 0.02


class OtaChannel:
    """Serialized request/response exchanges with a hidapi device handle."""

    def __init__(self, handle: Any, io_lock: threading.Lock) -> None:
        self._handle = handle
        self._io_lock = io_lock
        self._ota_lock = threading.Lock()

    def send_command(self, command: str, timeout: float = 3.0) -> Any:
        """Send a text command and return its parsed response.

        Returns the JSON-decoded response if the reply is JSON, the raw
        string otherwise, or ``None`` for plain acks. Raises
        :class:`CommandError` on NACK/empty replies and
        :class:`CommandTimeoutError` if no complete response arrives in time.
        """
        with self._ota_lock:
            deadline = time.monotonic() + timeout
            for chunk in build_ota_chunks(command):
                with self._io_lock:
                    self._handle.send_feature_report(chunk)
            return self._read_response(deadline, command)

    def _read_response(self, deadline: float, command: str) -> Any:
        remaining = 1
        data = b""
        while remaining > 0:
            if time.monotonic() > deadline:
                raise CommandTimeoutError(f"No response to {command!r}")
            with self._io_lock:
                resp = self._handle.get_feature_report(OTA_REPORT_ID, REPORT_LEN + 1)
            if not resp or len(resp) < OTA_READ_HEADER_LEN:
                time.sleep(_POLL_INTERVAL)
                continue
            resp = bytes(resp)
            msg_type = resp[3]
            length = resp[4] | (resp[5] << 8)
            payload = resp[OTA_READ_HEADER_LEN:]
            chunk = payload[: min(length, len(payload))]
            remaining = 0

            if msg_type in (OtaMsgType.FIRST, OtaMsgType.MID):
                data += chunk
                remaining = max(0, length - len(chunk))
            elif msg_type == OtaMsgType.ACK_NRSP:
                pass
            elif msg_type == OtaMsgType.ACK_RSP:
                remaining = 1  # another packet follows
            elif msg_type == OtaMsgType.NACK:
                text = chunk.decode("utf-8", errors="replace").strip()
                raise CommandError(text or f"Device returned NACK for {command!r}")
            elif msg_type == OtaMsgType.EMPTY:
                raise CommandError(f"Empty response from device for {command!r}")
            elif msg_type == OtaMsgType.RESULT:
                data = chunk
            else:
                raise CommandError(f"Unknown OTA message type: {msg_type}")

        if not data:
            return None
        text = data.decode("utf-8", errors="replace")
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return text
