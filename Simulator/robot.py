"""
robot.py  –  Top-level Robot simulation object

Owns all subsystems, the autonomous scheduler, and advances simulation time.
Mirrors the C++ Robot class.
"""

from __future__ import annotations

import math
import random
from enum import Enum, auto

from . import config as C
from .subsystems import Drive, Miner, Shooter
from .autonomous import (
    AutonomousRoutine,
    AutoStep,
    DeployRampStep,
    DriveDistanceStep,
    DriveLineUntilWallStep,
    DriveRadiusAngleStep,
    MineBlockStep,
)


class RobotMode(Enum):
    AWAIT = auto()
    SERIAL_TEST = auto()
    AUTONOMOUS = auto()


class Robot:
    """Top-level simulation orchestrator — mirrors the C++ Robot class."""

    def __init__(self):
        self.drive = Drive()
        self.miner = Miner()
        self.shooter = Shooter()
        self.autonomous = AutonomousRoutine()
        self.mode = RobotMode.AWAIT

        self.sim_time: float = 0.0  # seconds elapsed
        self._dt: float = C.SIM_DT

        # Telemetry history for plotting
        self.history_t: list[float] = []
        self.history_x: list[float] = []
        self.history_y: list[float] = []
        self.history_heading: list[float] = []
        self.history_true_x: list[float] = []
        self.history_true_y: list[float] = []
        self.history_vel_l: list[float] = []
        self.history_vel_r: list[float] = []
        self.history_miner_angle: list[float] = []
        self.history_shooter_pos: list[float] = []

    def init(self):
        self.drive.init()
        self.miner.init()
        self.shooter.init()
        self.sim_time = 0.0

    def load_default_autonomous(self):
        """Load the default autonomous routine matching handleGlobalCommand 'A'."""
        self.autonomous.clear()
        speed = 15.0
        self.autonomous.add(DriveDistanceStep(self.drive, 18, speed))
        self.autonomous.add(DriveRadiusAngleStep(self.drive, speed, -20, 45))
        self.autonomous.add(DriveRadiusAngleStep(self.drive, speed, 20, -45))
        self.autonomous.add(DriveDistanceStep(self.drive, 23, speed))
        self.autonomous.add(DriveRadiusAngleStep(self.drive, speed * 0.5, -15, 90))
        self.autonomous.add(DriveDistanceStep(self.drive, 2, speed * 0.25))
        self.autonomous.add(DriveLineUntilWallStep(self.drive, 0))
        self.autonomous.add(DeployRampStep(self.miner))
        self.autonomous.add(MineBlockStep(self.miner, 1000))

    def start_autonomous(self):
        self.mode = RobotMode.AUTONOMOUS
        self.autonomous.start()

    def stop_all(self):
        self.drive.stop()
        self.miner.store()
        self.shooter.stop()
        self.autonomous.stop()
        self.mode = RobotMode.AWAIT

    def step(self, dt: float | None = None):
        """Advance the simulation by one timestep."""
        if dt is None:
            # Vary loop time randomly between SIM_DT_MIN and SIM_DT_MAX
            # to simulate realistic Arduino loop jitter (5–8 ms)
            dt = random.uniform(C.SIM_DT_MIN, C.SIM_DT_MAX)
        self.sim_time += dt

        # Update subsystems
        self.drive.update(dt)
        self.miner.update(dt)
        self.shooter.update(dt)

        # Autonomous scheduler
        if self.mode in (RobotMode.AUTONOMOUS, RobotMode.SERIAL_TEST):
            self.autonomous.update()

        # Record telemetry
        self._record()

    def _record(self):
        pose = self.drive.get_pose()
        true_pose = self.drive.get_true_pose()
        self.history_t.append(self.sim_time)
        self.history_x.append(pose.x)
        self.history_y.append(pose.y)
        self.history_heading.append(pose.heading)
        self.history_true_x.append(true_pose.x)
        self.history_true_y.append(true_pose.y)

        l_vel = self.drive.left_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        r_vel = self.drive.right_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        self.history_vel_l.append(l_vel)
        self.history_vel_r.append(r_vel)
        self.history_miner_angle.append(self.miner.miner_servo.current_angle)
        self.history_shooter_pos.append(self.shooter.position)

    def clear_history(self):
        for lst in (self.history_t, self.history_x, self.history_y,
                    self.history_heading, self.history_true_x, self.history_true_y,
                    self.history_vel_l, self.history_vel_r,
                    self.history_miner_angle, self.history_shooter_pos):
            lst.clear()

    # ── Motor load estimation ────────────────────────────────────────────────
    @property
    def total_current_draw(self) -> float:
        """Estimated total current draw from all motors (A)."""
        return (self.drive.left_motor.current_draw_a
                + self.drive.right_motor.current_draw_a
                + self.shooter.motor.current_draw_a)

    @property
    def battery_voltage(self) -> float:
        return C.BATTERY_VOLTAGE

    @property
    def shooter_load_signal(self) -> float:
        """Extrapolated shooter load based on stall signal ratio."""
        if self.shooter.motor.current_draw_a < 0.01:
            return 0.0
        ratio = self.shooter.motor.current_draw_a / C.MOTOR_STALL_CURRENT_A
        return ratio * C.MAX_SIGNAL
