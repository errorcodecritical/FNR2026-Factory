"""Waypoint loading and lookup helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import yaml


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = 'map'

    @property
    def xy(self) -> Tuple[float, float]:
        return (self.x, self.y)


class WaypointStore:
    def __init__(self, waypoints: Iterable[Waypoint], aliases: Dict[str, str]):
        self._waypoints = {waypoint.name: waypoint for waypoint in waypoints}
        self._aliases = dict(aliases)

    def resolve(self, name: str) -> Optional[Waypoint]:
        key = name.strip()
        canonical = self._aliases.get(key, key)
        return self._waypoints.get(canonical)

    def xy(self, name: str) -> Optional[Tuple[float, float]]:
        waypoint = self.resolve(name)
        if waypoint is None:
            return None
        return waypoint.xy

    def names(self) -> Tuple[str, ...]:
        return tuple(sorted(self._waypoints))


def load_waypoints_file(path: str, default_frame: str = 'map') -> WaypointStore:
    waypoint_path = Path(path)
    if not waypoint_path.exists():
        raise FileNotFoundError(f'waypoint file does not exist: {path}')

    with waypoint_path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    raw_waypoints = data.get('waypoints', data)
    if not isinstance(raw_waypoints, dict):
        raise ValueError('waypoint file must contain a "waypoints" mapping')

    waypoints = []
    aliases: Dict[str, str] = {}

    for name, value in raw_waypoints.items():
        waypoint, waypoint_aliases = _waypoint_from_yaml(
            str(name),
            value,
            default_frame=default_frame,
        )
        waypoints.append(waypoint)
        for alias in waypoint_aliases:
            aliases[alias] = waypoint.name

    return WaypointStore(waypoints, aliases)


def _waypoint_from_yaml(
    name: str,
    value: object,
    default_frame: str,
) -> Tuple[Waypoint, Tuple[str, ...]]:
    if not isinstance(value, dict):
        raise ValueError(f'waypoint {name} must be a mapping')

    try:
        x = float(value['x'])
        y = float(value['y'])
    except KeyError as exc:
        raise ValueError(f'waypoint {name} requires x and y') from exc

    yaw = _parse_yaw(value)
    frame_id = str(value.get('frame_id', default_frame))
    raw_aliases = value.get('aliases', [])
    if isinstance(raw_aliases, str):
        raw_aliases = [raw_aliases]
    aliases = tuple(
        str(alias).strip() for alias in raw_aliases if str(alias).strip()
    )

    return Waypoint(name=name, x=x, y=y, yaw=yaw, frame_id=frame_id), aliases


def _parse_yaw(value: Dict[str, object]) -> float:
    if 'yaw' in value:
        return float(value['yaw'])
    if 'theta' in value:
        return float(value['theta'])
    if 'yaw_deg' in value:
        return math.radians(float(value['yaw_deg']))
    return 0.0
