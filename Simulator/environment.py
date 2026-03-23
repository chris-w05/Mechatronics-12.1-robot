"""
environment.py  –  Field / arena model for environmental effects

The robot operates on a game field with walls, lines, and crafting stations.
This module computes simulated sensor readings based on robot pose and
user-defined obstacles.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .physics import Pose2D


@dataclass
class Wall:
    """Axis-aligned wall segment."""
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class Line:
    """A line on the field that the line sensor can detect.
    Represented as a polyline (list of (x, y) waypoints)."""
    points: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class FieldEnvironment:
    """Holds the arena geometry and computes simulated sensor values.

    Default field dimensions are based on the ME EN 3230 competition arena
    (roughly 48×48 inches).
    """
    width: float = 48.0   # inches
    height: float = 48.0  # inches

    walls: list[Wall] = field(default_factory=list)
    lines: list[Line] = field(default_factory=list)

    def __post_init__(self):
        if not self.walls:
            # Default arena boundary walls
            self.walls = [
                Wall(0, 0, self.width, 0),          # bottom
                Wall(self.width, 0, self.width, self.height),  # right
                Wall(0, self.height, self.width, self.height), # top
                Wall(0, 0, 0, self.height),          # left
            ]
        if not self.lines:
            # Default tape lines — a simple line from start area to the mine
            self.lines = [
                Line([(6, 0), (6, self.height)]),     # vertical line near left
                Line([(0, 24), (self.width, 24)]),    # horizontal centre line
            ]

    def distance_to_wall_ahead(self, pose: Pose2D) -> float:
        """Compute the distance (inches) from the sensor (front of robot)
        to the nearest wall along the heading direction.

        The IR sensor is mounted at the front, roughly at (x + cos(θ)*1.5/2.54,
        y + sin(θ)*1.5/2.54) offset — but for simplicity we ray-cast from robot
        centre.
        """
        ox, oy = pose.x, pose.y
        dx = math.cos(pose.heading)
        dy = math.sin(pose.heading)

        min_dist = 1e6
        for wall in self.walls:
            d = self._ray_segment_intersection(ox, oy, dx, dy,
                                                wall.x1, wall.y1,
                                                wall.x2, wall.y2)
            if d is not None and d < min_dist:
                min_dist = d
        return min_dist

    def line_sensor_offset(self, pose: Pose2D,
                            sensor_offset_forward: float = 3.0) -> float:
        """Compute the line sensor reading (cm) given robot pose.

        Returns the lateral offset of the nearest tape line at the sensor location.
        Positive = line is to the right, negative = to the left, 0 = centred.
        Returns a large value if no line is nearby (simulating off-tape).
        """
        # Sensor is offset forward by LINESENSOR_LOCATION inches from wheel centre
        sx = pose.x + sensor_offset_forward * math.cos(pose.heading)
        sy = pose.y + sensor_offset_forward * math.sin(pose.heading)

        # Robot-local right direction
        rx = -math.sin(pose.heading)
        ry = math.cos(pose.heading)

        min_offset = 100.0  # large = no line detected
        for line in self.lines:
            for i in range(len(line.points) - 1):
                px, py = line.points[i]
                qx, qy = line.points[i + 1]
                d, lateral = self._point_to_segment_offset(
                    sx, sy, rx, ry, px, py, qx, qy
                )
                if d < 2.0 and abs(lateral) < abs(min_offset):
                    min_offset = lateral
        # Convert inches to cm
        return min_offset * 2.54

    # ── Geometry helpers ─────────────────────────────────────────────────────

    @staticmethod
    def _ray_segment_intersection(ox, oy, dx, dy, x1, y1, x2, y2) -> Optional[float]:
        """Ray-segment intersection.  Returns distance along ray or None."""
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            return None
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
        if t >= 0 and 0 <= u <= 1:
            return t
        return None

    @staticmethod
    def _point_to_segment_offset(sx, sy, rx, ry, px, py, qx, qy):
        """Compute perpendicular distance from point (sx,sy) to segment (p→q),
        and the lateral offset along local-right (rx,ry)."""
        lx, ly = qx - px, qy - py
        length_sq = lx * lx + ly * ly
        if length_sq < 1e-12:
            ddx, ddy = sx - px, sy - py
            dist = math.sqrt(ddx * ddx + ddy * ddy)
            lateral = ddx * rx + ddy * ry
            return dist, lateral
        t = max(0, min(1, ((sx - px) * lx + (sy - py) * ly) / length_sq))
        near_x = px + t * lx
        near_y = py + t * ly
        ddx, ddy = sx - near_x, sy - near_y
        dist = math.sqrt(ddx * ddx + ddy * ddy)
        lateral = ddx * rx + ddy * ry
        return dist, lateral
