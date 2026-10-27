"""MediaPipe hand landmarks -> LEAP Hand joint targets (canonical radians).

The approach follows the friend's wrist-camera teleop (jimin-kr/leap_hand,
right_hand_camera_finger_pub_stable.py): bend angles from the landmark chain,
an exponential filter and a per-step limit, and safety caps tighter than the
URDF limits (PIP 60 deg, DIP 79.23 deg, spread +-10 deg, 350 mA). Instead of
one curl ratio per finger, every LEAP joint gets its own human angle here.

Landmarks are MediaPipe's 21 points (``hand_world_landmarks``, metres, any frame:
only angles are used). LEAP has no little finger; landmarks 17-20 only define
the palm.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "hand_tracking.yaml"
SIDES = ("right", "left")
#: MediaPipe indices: (MCP, PIP, DIP, tip) per finger, thumb (CMC, MCP, IP, tip).
FINGERS = {"index": (5, 6, 7, 8), "middle": (9, 10, 11, 12), "ring": (13, 14, 15, 16)}
THUMB = (1, 2, 3, 4)
WRIST, PINKY_MCP = 0, 17
#: Canonical LEAP joint suffix -> what it is (r_hj_index_1 ... from the urdf limits).
JOINTS = (
    "thumb_1", "thumb_2", "thumb_3", "thumb_4",
    "index_1", "index_2", "index_3", "index_4",
    "middle_1", "middle_2", "middle_3", "middle_4",
    "ring_1", "ring_2", "ring_3", "ring_4",
)
#: joint suffix ending -> role in the config (_1 MCP flexion, _2 spread, _3 PIP, _4 DIP).
ROLE = {"1": "mcp", "2": "spread", "3": "pip", "4": "dip"}


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("degenerate hand landmarks")
    return v / n


def _bend(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """0 when a-b-c is straight, growing as the chain folds at b."""
    u, w = _unit(b - a), _unit(c - b)
    return math.acos(max(-1.0, min(1.0, float(u @ w))))


def palm_frame(p: np.ndarray, side: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(forward, thumbward, normal) unit vectors of the palm.

    forward: wrist -> middle MCP; thumbward: in the palm plane, toward the index side;
    normal: toward the palm side, where the fingers fold.
    """
    forward = _unit(p[9] - p[WRIST])
    across = p[5] - p[PINKY_MCP]  # little-finger side -> index side
    thumbward = _unit(across - (across @ forward) * forward)
    normal = np.cross(forward, thumbward)
    # Same physical "palm side" for both hands: a left hand is the mirror image.
    return forward, thumbward, (normal if side == "right" else -normal)


def human_angles(landmarks, side: str) -> dict[str, float]:
    """The operator's angle for every LEAP joint suffix, radians, 0 = flat open hand."""
    p = np.asarray(landmarks, dtype=float).reshape(21, 3)
    forward, thumbward, normal = palm_frame(p, side)
    out = {}
    for finger, (mcp, pip, dip, tip) in FINGERS.items():
        d = _unit(p[pip] - p[mcp])
        d_plane = d - (d @ normal) * normal
        # MCP flexion: the proximal phalanx leaving the palm plane toward the palm side
        # (independent of the spread; good to 90 deg, past the 82.85 deg cap).
        out[f"{finger}_1"] = math.atan2(float(d @ normal), float(np.linalg.norm(d_plane)))
        out[f"{finger}_3"] = _bend(p[mcp], p[pip], p[dip])
        out[f"{finger}_4"] = _bend(p[pip], p[dip], p[tip])
        # + toward the thumb, measured from the palm's forward axis.
        out[f"{finger}_2"] = math.atan2(float(d_plane @ thumbward), float(d_plane @ forward))
    cmc, mcp, ip, tip = THUMB
    metacarpal = _unit(p[mcp] - p[cmc])
    # thumb_1: rotation toward the palm (opposition) = how far the metacarpal leaves
    # the palm plane on the palm side; thumb_2: abduction away from the index finger.
    out["thumb_1"] = math.asin(max(-1.0, min(1.0, float(metacarpal @ normal))))
    in_plane = metacarpal - (metacarpal @ normal) * normal
    out["thumb_2"] = math.atan2(float(in_plane @ thumbward), float(in_plane @ forward))
    out["thumb_3"] = _bend(p[cmc], p[mcp], p[ip])
    out["thumb_4"] = _bend(p[mcp], p[ip], p[tip])
    return out


@dataclass(frozen=True)
class JointMap:
    scale: float
    offset: float  # human angle that maps to robot 0
    lower: float
    upper: float


def load_config(path: str | Path | None = None) -> dict:
    return yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text())


def joint_maps(config: dict, limits: dict[str, tuple[float, float]] | None = None) -> dict[str, JointMap]:
    """Per joint suffix: the config's role/thumb entry, caps intersected with URDF *limits*."""
    maps = {}
    for suffix in JOINTS:
        key = suffix if suffix.startswith("thumb") else ROLE[suffix[-1]]
        entry = {**config["joints"][key], **(config.get("overrides", {}).get(suffix) or {})}
        lower, upper = math.radians(entry["cap_deg"][0]), math.radians(entry["cap_deg"][1])
        if limits and suffix in limits:
            lower, upper = max(lower, limits[suffix][0]), min(upper, limits[suffix][1])
        maps[suffix] = JointMap(float(entry.get("scale", 1.0)), math.radians(entry.get("offset_deg", 0.0)),
                                lower, upper)
    return maps


def urdf_limits(urdf: str | Path, side: str) -> dict[str, tuple[float, float]]:
    """Joint suffix -> (lower, upper) of one hand in the canonical URDF."""
    import xml.etree.ElementTree as ET

    prefix = f"{side[0]}_hj_"
    out = {}
    for joint in ET.parse(urdf).getroot().findall("joint"):
        name, limit = joint.get("name", ""), joint.find("limit")
        if name.startswith(prefix) and limit is not None and name[len(prefix):] in JOINTS:
            out[name[len(prefix):]] = (float(limit.get("lower")), float(limit.get("upper")))
    return out


def to_robot(angles: dict[str, float], maps: dict[str, JointMap]) -> dict[str, float]:
    return {s: min(m.upper, max(m.lower, m.scale * (angles[s] - m.offset))) for s, m in maps.items()}


class Smoother:
    """Exponential filter, then at most ``max_step`` rad per update (friend's stable publisher)."""

    def __init__(self, alpha: float, max_step: float) -> None:
        self.alpha, self.max_step = float(alpha), float(max_step)
        self.filtered: dict[str, float] | None = None
        self.output: dict[str, float] | None = None

    def reset(self, start: dict[str, float]) -> None:
        self.filtered, self.output = dict(start), dict(start)

    def __call__(self, target: dict[str, float]) -> dict[str, float]:
        if self.output is None:
            self.reset(target)
            return dict(target)
        self.filtered = {k: self.filtered[k] + self.alpha * (v - self.filtered[k]) for k, v in target.items()}
        self.output = {k: self.output[k] + max(-self.max_step, min(self.max_step, v - self.output[k]))
                       for k, v in self.filtered.items()}
        return dict(self.output)


def synthetic_hand(side: str = "right", curl: float = 0.0, spread: float = 0.0,
                   thumb: float = 0.0) -> np.ndarray:
    """21 landmarks of a simple hand: every finger joint bent by *curl* rad, fingers fanned
    by *spread* rad, thumb opposed by *thumb* rad. Palm in the x-y plane, fingers along +y,
    normal (palm side) -z for a right hand. For tests and ``--test``."""
    p = np.zeros((21, 3))
    mirror = 1.0 if side == "right" else -1.0
    bases = {"index": 0.025, "middle": 0.0, "ring": -0.025}
    lengths = (0.045, 0.025, 0.02)
    fan = {"index": spread, "middle": 0.0, "ring": -spread}
    for finger, (mcp, pip, dip, tip) in FINGERS.items():
        p[mcp] = (mirror * bases[finger], 0.09, 0.0)
        heading = fan[finger]
        direction = np.array([mirror * math.sin(heading), math.cos(heading), 0.0])
        point, pitch = p[mcp].copy(), 0.0
        for index, length in zip((pip, dip, tip), lengths):
            pitch += curl  # each joint folds toward the palm side (-z)
            step = direction * math.cos(pitch) + np.array([0.0, 0.0, -math.sin(pitch)])
            point = point + length * step
            p[index] = point
    p[PINKY_MCP] = (mirror * -0.045, 0.08, 0.0)
    for k, offset in zip(range(18, 21), (0.03, 0.05, 0.065)):
        p[k] = p[PINKY_MCP] + (0.0, offset, 0.0)
    # Thumb: metacarpal along the index side, rotated out of the palm toward the palm side.
    p[1] = (mirror * 0.02, 0.02, 0.0)
    metacarpal = np.array([mirror * math.cos(0.6) * math.cos(thumb), math.sin(0.6) * math.cos(thumb),
                           -math.sin(thumb)])
    p[2] = p[1] + 0.035 * metacarpal
    p[3] = p[2] + 0.03 * metacarpal
    p[4] = p[3] + 0.025 * metacarpal
    return p
