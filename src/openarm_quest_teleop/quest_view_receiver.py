"""TCP client for quest_view's pose stream, and the choice of Quest input.

quest_view (``openarm_quest_view.server``) serves the headset page and
re-publishes its poses as HandUMI frames, one JSON object per line, on
TCP 127.0.0.1:65432. :class:`QuestViewReceiver` connects there (and again
after a drop) and keeps the newest frame, with the same interface as
:class:`udp_receiver.JsonUdpReceiver`, so the bridge and the monitor take
either. ``quest.source`` in the config picks one (:func:`open_source`).
"""

from __future__ import annotations

import collections
import socket
import threading
import time
from typing import Any, Callable

from .packet import parse_handumi, parse_packet
from .udp_receiver import JsonUdpReceiver, ReceivedPacket, parse_datagram

SOURCES = ("quest_view", "udp")
RECONNECT_SEC = 1.0


class QuestViewReceiver:
    """Background thread reading quest_view's TCP pose stream; keeps the latest frame."""

    def __init__(self, host: str = "127.0.0.1", port: int = 65432,
                 on_packet: Callable[[ReceivedPacket], None] | None = None) -> None:
        self._host, self._port = host, port
        self._on_packet = on_packet
        self._lock = threading.Lock()
        self._latest: ReceivedPacket | None = None
        self._recv_ts: collections.deque[int] = collections.deque(maxlen=512)
        self._sequence = 0
        self._malformed = 0
        self._error: str | None = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def latest(self) -> ReceivedPacket | None:
        with self._lock:
            return self._latest

    def counts(self) -> tuple[int, int]:
        with self._lock:
            return self._sequence, self._malformed

    def bind_error(self) -> str | None:
        """Why there is no stream now (quest_view not running, ...), or None."""
        with self._lock:
            return self._error

    def drain_recv_timestamps(self) -> list[int]:
        with self._lock:
            items = list(self._recv_ts)
            self._recv_ts.clear()
            return items

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=2.5)

    def _accept(self, line: bytes) -> None:
        recv_ns = time.time_ns()
        message = parse_datagram(line)
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
                with socket.create_connection((self._host, self._port), timeout=1.0) as sock:
                    sock.settimeout(0.5)
                    with self._lock:
                        self._error = None
                    buffer = b""
                    while self._running:
                        try:
                            chunk = sock.recv(65536)
                        except (TimeoutError, socket.timeout):
                            continue
                        if not chunk:
                            raise ConnectionError("quest_view closed the stream")
                        *lines, buffer = (buffer + chunk).split(b"\n")
                        for line in lines:
                            if line.strip():
                                self._accept(line)
            except OSError as error:
                with self._lock:
                    self._error = f"{self._host}:{self._port}: {error}"
                if self._running:
                    time.sleep(RECONNECT_SEC)


def source_of(quest: dict[str, Any]) -> str:
    source = quest.get("source", "udp")
    if source not in SOURCES:
        raise ValueError(f"quest.source must be one of {SOURCES}, not {source!r}")
    return source


def parser_for(quest: dict[str, Any]):
    """The packet parser of the config's ``quest.source``."""
    return parse_handumi if source_of(quest) == "quest_view" else parse_packet


def open_source(quest: dict[str, Any], host: str | None = None, port: int | None = None,
                on_packet: Callable[[ReceivedPacket], None] | None = None):
    """(receiver, parse, label) for the config's ``quest.source`` (default udp)."""
    if source_of(quest) == "quest_view":
        settings = quest["quest_view"]
        host, port = host or settings["host"], port or int(settings["port"])
        return (QuestViewReceiver(host, port, on_packet), parse_handumi,
                f"quest_view TCP {host}:{port}")
    host, port = host or quest["udp"]["host"], port or int(quest["udp"]["port"])
    return JsonUdpReceiver(host, port, on_packet=on_packet), parse_packet, f"UDP {host}:{port}"
