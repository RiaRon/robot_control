# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Derived from dora-openarm-vr, src/dora_openarm_vr/udp_receiver.py, at commit
# 072ce98d9c1d781e4b42626639c48f5c8f2ba8ee. The wire format (one UTF-8 JSON
# object per UDP datagram) and the drain-to-freshest loop are upstream's.
# Changes made here (KUKU Robot Lab, 2026-10-01):
#   - the latest packet is kept together with its own arrival time and a
#     sequence number, so a reader can tell a new packet from a repeated one
#     and compute its age (upstream kept arrival times in a separate queue);
#   - a datagram that is not a JSON object, or that spells NaN/Infinity, is
#     counted as malformed and dropped instead of being handed on;
#   - received and malformed datagrams are counted;
#   - an optional callback sees every accepted packet (used for recording);
#   - close() joins the thread, and a bind failure is reported once.

"""UDP receiver for the Quest teleoperation app's JSON packets."""

from __future__ import annotations

import collections
from dataclasses import dataclass
import json
import select
import socket
import threading
import time
from typing import Callable

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5006


@dataclass(frozen=True)
class ReceivedPacket:
    """One accepted datagram.

    ``recv_ns`` is this PC's wall clock when the datagram was read. It is not
    the headset's clock (that is the packet's own ``t``) and not a robot-state
    stamp.
    """

    sequence: int
    recv_ns: int
    message: dict


def _reject_constant(name: str) -> float:
    raise ValueError(f"non-finite JSON constant {name}")


def parse_datagram(data: bytes) -> dict | None:
    """Return the datagram's JSON object, or None if it is not one."""
    try:
        line = data.decode("utf-8", errors="strict").strip()
        if not line:
            return None
        parsed = json.loads(line, parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError):
        return None
    return parsed if isinstance(parsed, dict) else None


class JsonUdpReceiver:
    """Background thread that binds a UDP socket and keeps the latest packet."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        buf_size: int = 4096,
        on_packet: Callable[[ReceivedPacket], None] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._buf_size = buf_size
        self._on_packet = on_packet
        self._lock = threading.Lock()
        self._latest: ReceivedPacket | None = None
        self._recv_ts: collections.deque[int] = collections.deque(maxlen=512)
        self._sequence = 0
        self._malformed = 0
        self._bind_error: str | None = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def latest(self) -> ReceivedPacket | None:
        with self._lock:
            return self._latest

    def counts(self) -> tuple[int, int]:
        """Return (accepted, malformed) datagram counts since start."""
        with self._lock:
            return self._sequence, self._malformed

    def bind_error(self) -> str | None:
        with self._lock:
            return self._bind_error

    def drain_recv_timestamps(self) -> list[int]:
        """Return and clear the arrival timestamps (ns) collected since last call."""
        with self._lock:
            items = list(self._recv_ts)
            self._recv_ts.clear()
            return items

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=2.5)

    def _accept(self, data: bytes) -> None:
        recv_ns = time.time_ns()
        message = parse_datagram(data)
        with self._lock:
            if message is None:
                self._malformed += 1
                return
            self._sequence += 1
            packet = ReceivedPacket(self._sequence, recv_ns, message)
            self._recv_ts.append(recv_ns)
            self._latest = packet
        if self._on_packet is not None:
            self._on_packet(packet)

    def _loop(self) -> None:
        while self._running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as srv:
                    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    srv.bind((self._host, self._port))
                    srv.settimeout(0.5)
                    with self._lock:
                        self._bind_error = None

                    while self._running:
                        try:
                            data, _ = srv.recvfrom(self._buf_size)
                        except (TimeoutError, socket.timeout):
                            continue
                        self._accept(data)
                        # Drain any queued datagrams, so the latest packet is
                        # the freshest one and none is counted late.
                        while select.select([srv], [], [], 0.0)[0]:
                            data, _ = srv.recvfrom(self._buf_size)
                            self._accept(data)
            except OSError as error:
                with self._lock:
                    self._bind_error = str(error)
                if self._running:
                    time.sleep(1.0)
