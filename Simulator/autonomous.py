"""
autonomous.py  –  Autonomous step framework + concrete steps

Mirrors the C++ autonomous routine / step system.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from .subsystems import Drive, Miner, Shooter
from . import config as C


# ═════════════════════════════════════════════════════════════════════════════
#  AutoStep base
# ═════════════════════════════════════════════════════════════════════════════

class AutoStep:
    """Abstract base — mirrors the C++ AutoStep interface."""

    def start(self): ...
    def update(self): ...
    def is_finished(self) -> bool: return True
    def end(self): ...
    @property
    def name(self) -> str:
        return type(self).__name__


# ═════════════════════════════════════════════════════════════════════════════
#  AutonomousRoutine
# ═════════════════════════════════════════════════════════════════════════════

class AutonomousRoutine:
    """Sequential step runner — mirrors C++ AutonomousRoutine."""

    def __init__(self):
        self._steps: list[AutoStep] = []
        self._index: int = 0
        self._running: bool = False

    def add(self, step: AutoStep):
        self._steps.append(step)

    def start(self):
        self._index = 0
        self._running = True
        if self._steps:
            self._steps[0].start()

    def update(self):
        if not self._running or self._index >= len(self._steps):
            return
        step = self._steps[self._index]
        step.update()
        if step.is_finished():
            step.end()
            self._index += 1
            if self._index < len(self._steps):
                self._steps[self._index].start()

    def stop(self):
        if self._index < len(self._steps):
            self._steps[self._index].end()
        self._index = len(self._steps)
        self._running = False

    def reset(self):
        if self._running and self._index < len(self._steps):
            self._steps[self._index].end()
        self._index = 0

    def clear(self):
        self.stop()
        self._steps.clear()

    @property
    def is_complete(self) -> bool:
        return self._index >= len(self._steps)

    @property
    def current_step_name(self) -> str:
        if 0 <= self._index < len(self._steps):
            return self._steps[self._index].name
        return "(done)"

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def step_count(self) -> int:
        return len(self._steps)


# ═════════════════════════════════════════════════════════════════════════════
#  Concrete steps
# ═════════════════════════════════════════════════════════════════════════════

class DriveDistanceStep(AutoStep):
    def __init__(self, drive: Drive, distance: float, velocity: float):
        self._drive = drive
        self._target = distance
        self._velocity = velocity
        self._start_dist = 0.0

    def start(self):
        self._start_dist = self._drive.get_distance()
        self._drive.set_speed(self._velocity)

    def is_finished(self) -> bool:
        return abs(self._drive.get_distance() - self._start_dist) >= abs(self._target)

    def end(self):
        self._drive.set_speed(0)

    @property
    def name(self):
        return f"DriveDistance({self._target:.1f}in @ {self._velocity:.1f}in/s)"


class DriveRadiusAngleStep(AutoStep):
    def __init__(self, drive: Drive, velocity: float, radius: float, angle_deg: float):
        self._drive = drive
        self._velocity = velocity
        self._radius = radius
        self._target_angle = angle_deg * math.pi / 180.0
        self._start_angle = 0.0

    def start(self):
        self._start_angle = self._drive.get_accumulated_heading()
        self._drive.follow_radius_at_velocity(self._velocity, self._radius)

    def is_finished(self) -> bool:
        return abs(self._drive.get_accumulated_heading() - self._start_angle) >= abs(self._target_angle)

    def end(self):
        self._drive.set_speed(0)

    @property
    def name(self):
        return f"DriveRadiusAngle(r={self._radius:.1f}, {self._target_angle*180/math.pi:.0f}°)"


class FollowLineStepAuto(AutoStep):
    def __init__(self, drive: Drive, distance: float, velocity: int):
        self._drive = drive
        self._target = distance
        self._velocity = velocity
        self._start_dist = 0.0

    def start(self):
        self._start_dist = self._drive.get_distance()
        self._drive.follow_line_hardset(self._velocity)

    def is_finished(self) -> bool:
        return (self._drive.get_distance() - self._start_dist) >= self._target

    def end(self):
        self._drive.set_speed(0)

    @property
    def name(self):
        return f"FollowLine({self._target:.1f}in)"


class DriveLineUntilWallStep(AutoStep):
    def __init__(self, drive: Drive, target_distance_cm: float):
        self._drive = drive
        self._target = target_distance_cm

    def start(self):
        self._drive.approach_along_line(self._target)

    def is_finished(self) -> bool:
        return abs(self._drive.get_distance_sensor_reading() - self._target / 2.54) < 0.5

    def end(self):
        pass  # drive continues holding distance

    @property
    def name(self):
        return f"DriveLineUntilWall({self._target:.1f}cm)"


class DriveUntilWallStep(AutoStep):
    def __init__(self, drive: Drive, target_distance_cm: float):
        self._drive = drive
        self._target = target_distance_cm

    def start(self):
        self._drive.approach_distance(self._target)

    def is_finished(self) -> bool:
        reading = self._drive.get_distance_sensor_reading()
        return abs(reading - self._target / 2.54) < 0.5 and abs(self._drive.get_avg_velocity()) < 1

    def end(self):
        self._drive.set_speed(0)

    @property
    def name(self):
        return f"DriveUntilWall({self._target:.1f}cm)"


class MineBlockStep(AutoStep):
    def __init__(self, miner: Miner, hits: int):
        self._miner = miner
        self._hits = hits

    def start(self):
        self._miner.start_mining(self._hits)

    def is_finished(self) -> bool:
        return self._miner.is_done_mining()

    def end(self):
        pass

    @property
    def name(self):
        return f"MineBlock({self._hits} hits)"


class FireStepAuto(AutoStep):
    def __init__(self, shooter: Shooter, time_ms: int):
        self._shooter = shooter
        self._time_s = time_ms / 1000.0
        self._start = 0.0
        self._sim_time = 0.0

    def start(self):
        self._start = self._sim_time
        self._shooter.fire()

    def update(self):
        pass  # time tracked externally

    def is_finished(self) -> bool:
        return (self._sim_time - self._start) > self._time_s

    def end(self):
        self._shooter.stop()


class DeployRampStep(AutoStep):
    def __init__(self, miner: Miner):
        self._miner = miner
        self._done = False

    def start(self):
        self._miner.deploy_ramp()
        self._done = True

    def is_finished(self) -> bool:
        return self._done

    @property
    def name(self):
        return "DeployRamp"


class DelayStepAuto(AutoStep):
    def __init__(self, delay_ms: int):
        self._delay_s = delay_ms / 1000.0
        self._start = 0.0
        self._elapsed = 0.0

    def start(self):
        self._elapsed = 0.0

    def update(self):
        pass

    def is_finished(self) -> bool:
        return self._elapsed >= self._delay_s

    @property
    def name(self):
        return f"Delay({self._delay_s*1000:.0f}ms)"


class CompositeStepAuto(AutoStep):
    """Sequential composition of steps."""
    def __init__(self, steps: list[AutoStep]):
        self._steps = steps
        self._index = 0

    def start(self):
        self._index = 0
        if self._steps:
            self._steps[0].start()

    def update(self):
        if self._index >= len(self._steps):
            return
        step = self._steps[self._index]
        step.update()
        if step.is_finished():
            step.end()
            self._index += 1
            if self._index < len(self._steps):
                self._steps[self._index].start()

    def is_finished(self) -> bool:
        return self._index >= len(self._steps)

    def end(self):
        if self._index < len(self._steps):
            self._steps[self._index].end()

    @property
    def name(self):
        if 0 <= self._index < len(self._steps):
            return f"Composite[{self._index}]:{self._steps[self._index].name}"
        return "Composite(done)"


class ParallelStepAuto(AutoStep):
    """Run steps in parallel. Finishes when ALL or ANY complete."""
    def __init__(self, steps: list[AutoStep], finish_any: bool = False):
        self._steps = steps
        self._finish_any = finish_any

    def start(self):
        for s in self._steps:
            s.start()

    def update(self):
        for s in self._steps:
            if not s.is_finished():
                s.update()

    def is_finished(self) -> bool:
        if self._finish_any:
            return any(s.is_finished() for s in self._steps)
        return all(s.is_finished() for s in self._steps)

    def end(self):
        for s in self._steps:
            s.end()

    @property
    def name(self):
        mode = "ANY" if self._finish_any else "ALL"
        return f"Parallel({mode})"
