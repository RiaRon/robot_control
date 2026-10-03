"""Single ownership for the split and integrated OpenArm + RH56F1 bringups.

Two launch files must not own the same controller manager name or the same
physical device at once: the integrated launches put arms and hands in one
``/controller_manager``, the split ones give each its own. These checks run in
the launch process before any node starts.

``require_no_controller_manager``
    Refuses to start when a node named ``controller_manager`` already exists in
    the target namespace. Discovery-based, so a manager killed without a clean
    shutdown can linger in the graph for its DDS lease; the check waits for it
    to go before refusing.

``hold_device_lock``
    An exclusive, non-blocking ``flock`` on ``<lock_dir>/<device>.lock``, held
    by the launch process until it exits (also on a crash). Two launches on the
    same machine, in the same ``lock_dir``, can then never both drive the same
    CAN interface. It does not span machines or containers with separate
    ``/tmp``; ``OPENARM_RH56F1_LOCK_DIR`` moves it to a shared directory.
"""

from __future__ import annotations

import fcntl
import os
from pathlib import Path
import time

LOCK_DIR_ENV = "OPENARM_RH56F1_LOCK_DIR"
DEFAULT_LOCK_DIR = "/tmp/openarm_rh56f1_locks"
#: Long enough for a FastDDS participant lease to expire after a SIGKILL.
DEFAULT_WAIT_SEC = 25.0
DISCOVERY_SEC = 1.5

_held: list[int] = []


class OwnershipError(RuntimeError):
    pass


def _controller_manager_present(namespace: str, discovery_sec: float) -> bool:
    import rclpy
    from rclpy.context import Context

    context = Context()
    rclpy.init(context=context)
    try:
        node = rclpy.create_node("openarm_rh56f1_ownership_check", context=context,
                                 enable_rosout=False, start_parameter_services=False)
        try:
            deadline = time.monotonic() + discovery_sec
            wanted = "/" + namespace.strip("/") if namespace.strip("/") else "/"
            while time.monotonic() < deadline:
                for name, ns in node.get_node_names_and_namespaces():
                    if name == "controller_manager" and ns == wanted:
                        return True
                time.sleep(0.1)
            return False
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def require_no_controller_manager(namespace: str, wait_sec: float = DEFAULT_WAIT_SEC) -> None:
    deadline = time.monotonic() + wait_sec
    while _controller_manager_present(namespace, DISCOVERY_SEC):
        if time.monotonic() > deadline:
            where = f"/{namespace.strip('/')}/controller_manager" if namespace.strip("/") \
                else "/controller_manager"
            raise OwnershipError(
                f"{where} is already running. Only one launch may own it: stop the other "
                "bringup first (the integrated and the split bringups share /controller_manager).")
        time.sleep(1.0)


def hold_device_lock(device: str) -> Path:
    """Lock *device* for the life of this process, or raise OwnershipError."""
    directory = Path(os.environ.get(LOCK_DIR_ENV, DEFAULT_LOCK_DIR))
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{device.replace('/', '_')}.lock"
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(descriptor)
        raise OwnershipError(
            f"{device} is already owned by another launch (lock {path}). Stop that "
            "bringup first; two processes must never drive the same device.") from None
    os.ftruncate(descriptor, 0)
    os.write(descriptor, f"{os.getpid()}\n".encode())
    _held.append(descriptor)
    return path
