"""
subsystems.py  –  High-level subsystem simulations

Mirrors the C++ subsystem hierarchy:
  - Drive      (differential-drive, encoders, line sensor, distance sensor, odometry)
  - Miner      (servo-based press/retract cycle)
  - Shooter    (motor + encoder position control, autofire state machine)
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from . import config as C
from .physics import (
    EncoderSim,
    MotorModel,
    Odometry,
    PIDController,
    Pose2D,
    RamseteController,
    ServoModel,
)


# ═════════════════════════════════════════════════════════════════════════════
#  Drive subsystem
# ═════════════════════════════════════════════════════════════════════════════

class DriveMode(Enum):
    STRAIGHT = auto()
    ARC = auto()
    STOPPED = auto()
    LINEFOLLOWING = auto()
    LINEFOLLOWING_HARDSET = auto()
    LINEFOLLOWING_DISTANCE = auto()
    HARDSET = auto()
    DISTANCE = auto()


class Drive:
    """Simulates the differential-drive subsystem.

    User-injectable sensor values:
    - line_sensor_value:  position in cm (neg = left, pos = right, 0 = centred)
    - distance_sensor_cm: distance reading in cm from the IR sensor
    """

    def __init__(self):
        # Motors + encoders
        self.left_motor = MotorModel()
        self.right_motor = MotorModel()
        self.left_encoder = EncoderSim()
        self.right_encoder = EncoderSim()

        # PID controllers (motor-level)
        self.left_pid = PIDController(
            kp=C.DRIVE_L_PID["kp"], ki=C.DRIVE_L_PID["ki"], kd=C.DRIVE_L_PID["kd"],
            output_min=-C.MAX_SIGNAL, output_max=C.MAX_SIGNAL,
        )
        self.right_pid = PIDController(
            kp=C.DRIVE_R_PID["kp"], ki=C.DRIVE_R_PID["ki"], kd=C.DRIVE_R_PID["kd"],
            output_min=-C.MAX_SIGNAL, output_max=C.MAX_SIGNAL,
        )
        self.line_pid = PIDController(
            kp=C.DRIVE_LINEFOLLOW_GAINS["kp"],
            ki=C.DRIVE_LINEFOLLOW_GAINS["ki"],
            kd=C.DRIVE_LINEFOLLOW_GAINS["kd"],
        )
        self.distance_pid = PIDController(
            kp=C.DRIVE_DISTANCE_PID["kp"],
            ki=C.DRIVE_DISTANCE_PID["ki"],
            kd=C.DRIVE_DISTANCE_PID["kd"],
        )

        # Odometry
        self.odometry = Odometry()
        self.desired_odometry = Odometry()
        self.ramsete = RamseteController()

        # Mode / velocity state
        self.mode = DriveMode.HARDSET
        self._target_speed_l = 0.0
        self._target_speed_r = 0.0
        self._speed_l = 0.0  # ramped speeds
        self._speed_r = 0.0
        self._signal_l = 0
        self._signal_r = 0
        self._target_distance = 0.0    # cm for distance mode
        self._left_target_pos = 0.0    # in
        self._right_target_pos = 0.0   # in

        # ── User-injected sensor values ──
        self.line_sensor_value: float = 0.0     # cm offset
        self.distance_sensor_cm: float = 15.0   # cm

    def init(self):
        self.odometry.init(C.INITIAL_X, C.INITIAL_Y, C.INITIAL_HEADING)
        self.desired_odometry.init(C.INITIAL_X, C.INITIAL_Y, C.INITIAL_HEADING)
        self.left_pid.reset()
        self.right_pid.reset()

    def _sync_target_positions(self):
        self._left_target_pos = self.left_encoder.get_count() * C.DRIVETRAIN_TICKS_TO_IN
        self._right_target_pos = self.right_encoder.get_count() * C.DRIVETRAIN_TICKS_TO_IN

    # ── setpoint API (mirrors C++ Drive) ─────────────────────────────────────

    def set_speed(self, speed: float, reset_pid: bool = True):
        self.mode = DriveMode.STRAIGHT
        if reset_pid:
            self.left_pid.reset()
            self.right_pid.reset()
        self._target_speed_l = speed
        self._target_speed_r = speed

    def hard_set_speed(self, left: int, right: Optional[int] = None):
        self.mode = DriveMode.HARDSET
        if right is None:
            right = left
        self._signal_l = left
        self._signal_r = right

    def follow_radius_at_velocity(self, velocity: float, radius: float, reset_pid: bool = True):
        self.mode = DriveMode.ARC
        if reset_pid:
            self.left_pid.reset()
            self.right_pid.reset()
        half_w = C.DRIVETRAIN_WIDTH / 2.0
        if abs(radius) < 1e-6:
            self._target_speed_l = -velocity
            self._target_speed_r = velocity
            return
        omega = -velocity / radius
        self._target_speed_l = velocity - half_w * omega
        self._target_speed_r = velocity + half_w * omega

    def follow_line_hardset(self, speed: int):
        self.mode = DriveMode.LINEFOLLOWING_HARDSET
        self._signal_l = speed
        self._signal_r = speed

    def follow_line(self, speed: float):
        self.mode = DriveMode.LINEFOLLOWING
        self._speed_l = speed
        self._speed_r = speed

    def approach_distance(self, distance_cm: float):
        self.mode = DriveMode.DISTANCE
        self._target_distance = distance_cm

    def approach_along_line(self, distance_cm: float):
        self.mode = DriveMode.LINEFOLLOWING_DISTANCE
        self._target_distance = distance_cm

    def stop(self):
        self.mode = DriveMode.STOPPED
        self.hard_set_speed(0)

    # ── Queries ──────────────────────────────────────────────────────────────

    def get_pose(self) -> Pose2D:
        return self.desired_odometry.pose

    def get_true_pose(self) -> Pose2D:
        return self.odometry.pose

    def get_distance(self) -> float:
        return self.desired_odometry.distance_travelled

    def get_true_distance(self) -> float:
        return self.odometry.distance_travelled

    def get_accumulated_heading(self) -> float:
        return self.desired_odometry.accumulated_heading

    def get_true_accumulated_heading(self) -> float:
        return self.odometry.accumulated_heading

    def get_distance_sensor_reading(self) -> float:
        """Returns current distance in inches (converted from user-injected cm)."""
        return self.distance_sensor_cm / 2.54

    def get_avg_velocity(self) -> float:
        l_vel = self.left_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        r_vel = self.right_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        return (l_vel + r_vel) / 2.0

    # ── Main simulation step ─────────────────────────────────────────────────

    def update(self, dt: float):
        left_pos = self.left_encoder.get_count() * C.DRIVETRAIN_TICKS_TO_IN
        right_pos = self.right_encoder.get_count() * C.DRIVETRAIN_TICKS_TO_IN
        left_vel = self.left_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        right_vel = self.right_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN

        # Noisy sensor readings (what the control loop actually sees)
        line_sensor_noisy = self.line_sensor_value + random.gauss(0.0, C.LINE_SENSOR_NOISE_CM)
        distance_sensor_noisy = max(0.0, self.distance_sensor_cm + random.gauss(0.0, C.DISTANCE_SENSOR_NOISE_CM))

        sig_l = 0.0
        sig_r = 0.0

        if self.mode in (DriveMode.STRAIGHT, DriveMode.ARC, DriveMode.STOPPED):
            # Ramp speed toward target
            max_delta = 250.0 * dt
            delta_l = self._target_speed_l - self._speed_l
            delta_r = self._target_speed_r - self._speed_r
            max_change = max(abs(delta_l), abs(delta_r))
            if max_change > max_delta and max_change > 0:
                scale = max_delta / max_change
                self._speed_l += delta_l * scale
                self._speed_r += delta_r * scale
            else:
                self._speed_l = self._target_speed_l
                self._speed_r = self._target_speed_r

            vl_ref = max(-C.MAX_VELOCITY, min(C.MAX_VELOCITY, self._speed_l))
            vr_ref = max(-C.MAX_VELOCITY, min(C.MAX_VELOCITY, self._speed_r))

            v_ref = 0.5 * (vl_ref + vr_ref)
            w_ref = (vr_ref - vl_ref) / C.DRIVETRAIN_WIDTH

            v_cmd, w_cmd = self.ramsete.step(
                self.desired_odometry.pose, v_ref, w_ref, self.odometry.pose
            )

            half_w = C.DRIVETRAIN_WIDTH * 0.5
            vl_cmd = max(-C.MAX_VELOCITY, min(C.MAX_VELOCITY, v_cmd - half_w * w_cmd))
            vr_cmd = max(-C.MAX_VELOCITY, min(C.MAX_VELOCITY, v_cmd + half_w * w_cmd))

            self._left_target_pos += vl_cmd * dt
            self._right_target_pos += vr_cmd * dt

            sig_l = self.left_pid.update(left_pos, left_vel, self._left_target_pos, dt)
            sig_r = self.right_pid.update(right_pos, right_vel, self._right_target_pos, dt)

            # Advance feedforward trajectory
            self.desired_odometry.update_from_deltas(
                self._target_speed_l * dt, self._target_speed_r * dt
            )

        elif self.mode == DriveMode.LINEFOLLOWING_HARDSET:
            correction = line_sensor_noisy
            correction_signal = self.line_pid.update_simple(correction, 0, dt)
            sig_l = self._signal_l + correction_signal / C.LINESENSOR_LR_RATIO
            sig_r = self._signal_r - correction_signal

        elif self.mode == DriveMode.LINEFOLLOWING:
            correction = line_sensor_noisy * C.DRIVE_LINEFOLLOW_VELOCITY_GAIN * self._speed_l
            new_speed_l = self._speed_l + correction / C.LINESENSOR_LR_RATIO
            new_speed_r = self._speed_r - correction

            self._left_target_pos += new_speed_l * dt
            self._right_target_pos += new_speed_r * dt

            sig_l = self.left_pid.update(left_pos, left_vel, self._left_target_pos, dt)
            sig_r = self.right_pid.update(right_pos, right_vel, self._right_target_pos, dt)

        elif self.mode == DriveMode.LINEFOLLOWING_DISTANCE:
            dist_in = distance_sensor_noisy / 2.54
            signal_base = self.distance_pid.update_simple(dist_in, self._target_distance / 2.54, dt)
            self._signal_l = signal_base
            self._signal_r = signal_base
            correction = line_sensor_noisy
            correction_signal = self.line_pid.update_simple(correction, 0, dt)
            sig_l = self._signal_l + correction_signal / C.LINESENSOR_LR_RATIO
            sig_r = self._signal_r - correction_signal

        elif self.mode == DriveMode.HARDSET:
            sig_l = float(self._signal_l)
            sig_r = float(self._signal_r)

        elif self.mode == DriveMode.DISTANCE:
            dist_in = distance_sensor_noisy / 2.54
            sig_l = self.distance_pid.update_simple(dist_in, self._target_distance / 2.54, dt)
            sig_r = sig_l

        # Clamp signals
        sig_l = max(-C.MAX_SIGNAL, min(C.MAX_SIGNAL, sig_l))
        sig_r = max(-C.MAX_SIGNAL, min(C.MAX_SIGNAL, sig_r))

        # Step motors → wheel velocity (in/s)
        left_vel = self.left_motor.step(sig_l, dt=dt)
        right_vel_out = self.right_motor.step(sig_r, dt=dt)

        # Update encoders from wheel velocity
        self.left_encoder.update(left_vel, dt)
        self.right_encoder.update(right_vel_out, dt)

        # Update true-pose odometry from encoder ticks
        self.odometry.update_from_encoders(self.left_encoder, self.right_encoder)


# ═════════════════════════════════════════════════════════════════════════════
#  Miner subsystem
# ═════════════════════════════════════════════════════════════════════════════

class MinerMode(Enum):
    OFF = auto()
    MINING = auto()
    STORE = auto()
    LIFT = auto()


class Miner:
    def __init__(self):
        self.mode = MinerMode.STORE
        self.miner_servo = ServoModel(current_angle=C.MINER_SERVO_STORE_ANGLE,
                                       target_angle=C.MINER_SERVO_STORE_ANGLE)
        self.ramp_servo = ServoModel(current_angle=C.RAMP_SERVO_STORE_ANGLE,
                                      target_angle=C.RAMP_SERVO_STORE_ANGLE)
        self._hits_done: int = 0
        self._hits_target: int = 10
        self._cycle_start: float = 0.0
        self._on_start: float = 0.0
        self._mining_start: float = 0.0
        self._timed_out: bool = False
        self._max_continuous_s: float = 300.0   # 5 min
        self._sim_time: float = 0.0

    def init(self):
        self.mode = MinerMode.STORE
        self._hits_done = 0
        self._timed_out = False
        self._cycle_start = 0
        self._on_start = 0
        self._mining_start = 0

    def start_mining(self, hits: int = -1):
        if self.mode != MinerMode.MINING:
            self.mode = MinerMode.MINING
            self._hits_done = 0
            self._hits_target = hits
            self._cycle_start = 0
            self._on_start = 0
            self._mining_start = 0
            self._timed_out = False

    def stop_mining(self):
        self.mode = MinerMode.OFF
        self._cycle_start = 0
        self._on_start = 0
        self._mining_start = 0
        self.miner_servo.set_angle(C.MINER_SERVO_RETRACT_ANGLE)

    def store(self):
        self._hits_done = 0
        self.mode = MinerMode.STORE

    def deploy_ramp(self):
        self.ramp_servo.set_angle(C.RAMP_SERVO_PASSIVE_ANGLE)

    def is_done_mining(self) -> bool:
        return self.mode == MinerMode.OFF

    @property
    def hits_done(self) -> int:
        return self._hits_done

    def update(self, dt: float):
        self._sim_time += dt
        now = self._sim_time

        if self._hits_target != -1 and self._hits_done >= self._hits_target:
            self.stop_mining()

        if self._timed_out:
            self.miner_servo.set_angle(C.MINER_SERVO_RETRACT_ANGLE)
            self.miner_servo.update(dt)
            return

        if self.mode == MinerMode.LIFT:
            self.ramp_servo.set_angle(C.RAMP_SERVO_LIFT_ANGLE)
            self.ramp_servo.update(dt)
            return

        if self.mode == MinerMode.STORE:
            self.miner_servo.set_angle(C.MINER_SERVO_STORE_ANGLE)
            self.ramp_servo.set_angle(C.RAMP_SERVO_STORE_ANGLE)
            self.miner_servo.update(dt)
            self.ramp_servo.update(dt)
            return

        if self.mode == MinerMode.OFF:
            self.miner_servo.set_angle(C.MINER_SERVO_RETRACT_ANGLE)
            self.miner_servo.update(dt)
            return

        if self.mode == MinerMode.MINING:
            self._mine(now, dt)
            self.ramp_servo.set_angle(C.RAMP_SERVO_PASSIVE_ANGLE)
            self.ramp_servo.update(dt)

    def _mine(self, now: float, dt: float):
        cycle_s = C.MINER_CYCLE_MS / 1000.0
        press_s = C.MINER_PRESS_MS / 1000.0

        if self._mining_start == 0:
            self._mining_start = now

        if (self._hits_target == -1 and self._max_continuous_s > 0
                and (now - self._mining_start) >= self._max_continuous_s):
            self._timed_out = True
            self.stop_mining()
            return

        if self._cycle_start == 0:
            self._cycle_start = now
            self._on_start = now
            self.miner_servo.set_angle(C.MINER_SERVO_PRESS_ANGLE)
            self.miner_servo.update(dt)
            return

        elapsed = now - self._cycle_start
        elapsed_in_cycle = elapsed % cycle_s

        if elapsed_in_cycle < press_s:
            self.miner_servo.set_angle(C.MINER_SERVO_PRESS_ANGLE)
        else:
            self.miner_servo.set_angle(C.MINER_SERVO_RETRACT_ANGLE)
            if elapsed >= cycle_s:
                cycles_passed = int(elapsed / cycle_s)
                self._cycle_start += cycles_passed * cycle_s
                self._hits_done += cycles_passed
                if self._hits_target != -1 and self._hits_done >= self._hits_target:
                    self.stop_mining()

        self.miner_servo.update(dt)


# ═════════════════════════════════════════════════════════════════════════════
#  Shooter subsystem
# ═════════════════════════════════════════════════════════════════════════════

class ShooterMode(Enum):
    OFF = auto()
    POSITION = auto()
    VELOCITY = auto()
    TEST = auto()
    AUTO = auto()


class AutofireState(Enum):
    WAITFORBLOCK = auto()
    HASBLOCK = auto()
    FIRE = auto()


class Shooter:
    """Simulates the shooter subsystem.

    User-injectable sensor: block_switch (bool) — set to True when a block is present.
    """

    def __init__(self):
        # Output velocity in output-shaft rotations/sec
        self.motor = MotorModel(max_velocity=4.2, tau=0.02)
        self.pid = PIDController(
            kp=C.SHOOTER_POSITION_PID["kp"],
            ki=C.SHOOTER_POSITION_PID["ki"],
            kd=C.SHOOTER_POSITION_PID["kd"],
            ff_func=C.shooter_ff,
        )
        self.mode = ShooterMode.OFF
        self.autofire_state = AutofireState.WAITFORBLOCK

        self.position: float = 0.0     # rotations
        self.velocity: float = 0.0     # rotations/sec
        self.target_position: float = 0.0
        self.target_velocity: float = 0.0

        # ── User-injected sensor ──
        self.block_switch: bool = False
        self._last_switch: bool = False

        self._fire_time: float = 0.0
        self._block_detected_time: float = 0.0
        self._sim_time: float = 0.0

    def init(self):
        self.mode = ShooterMode.OFF
        self.pid.reset()

    def fire(self):
        if self.mode != ShooterMode.POSITION:
            self.mode = ShooterMode.POSITION
            self.pid.reset()
        self.target_position = self._next_fire_position()

    def prime(self):
        if self.mode != ShooterMode.POSITION:
            self.mode = ShooterMode.POSITION
            self.pid.reset()
        self.target_position = self._next_prime_position()

    def auto_fire(self):
        if self.mode != ShooterMode.AUTO:
            self.mode = ShooterMode.AUTO
            self.pid.reset()
        self.target_position = math.floor(self.target_position) + C.SHOOTER_PULL_BACK_ROTATIONS

    def stop_firing(self):
        self.mode = ShooterMode.OFF
        self.target_velocity = 0

    def hold_position(self, amount: float):
        if self.mode != ShooterMode.POSITION:
            self.mode = ShooterMode.POSITION
            self.pid.reset()
        self.target_position = math.floor(self.target_position) + amount

    def stop(self):
        self.stop_firing()

    def _next_prime_position(self) -> float:
        return math.floor(self.target_position) + C.SHOOTER_PULL_BACK_ROTATIONS

    def _next_fire_position(self) -> float:
        return math.ceil(self.target_position) + 0.12

    def update(self, dt: float):
        self._sim_time += dt

        # Integrate position from motor velocity (rotations/sec) with encoder noise
        self.position += self.motor.velocity * dt + random.gauss(0.0, C.SHOOTER_POSITION_NOISE_ROT)
        self.velocity = self.motor.velocity

        signal = 0.0
        if self.mode == ShooterMode.OFF:
            signal = 0
        elif self.mode == ShooterMode.TEST:
            signal = -130
        elif self.mode == ShooterMode.POSITION:
            signal = self.pid.update(self.position, self.velocity, self.target_position, dt)
            signal = max(-C.MAX_SIGNAL, min(C.MAX_SIGNAL, signal))
        elif self.mode == ShooterMode.AUTO:
            self._handle_autofire(dt)
            signal = self.pid.update(self.position, self.velocity, self.target_position, dt)
            signal = max(-C.MAX_SIGNAL, min(C.MAX_SIGNAL, signal))

        self.motor.step(signal, dt=dt)

    def _handle_autofire(self, dt: float):
        now = self._sim_time
        if self.autofire_state == AutofireState.WAITFORBLOCK:
            reading = self.block_switch
            if reading and reading != self._last_switch:
                self._block_detected_time = now
                self.autofire_state = AutofireState.HASBLOCK
            self._last_switch = reading

        elif self.autofire_state == AutofireState.HASBLOCK:
            if (now - self._block_detected_time) > C.SHOOTER_SETTLE_TIME_MS / 1000.0:
                self.target_position = self._next_fire_position()
                self.autofire_state = AutofireState.FIRE
                self._fire_time = now

        elif self.autofire_state == AutofireState.FIRE:
            if (now - self._fire_time) > C.SHOOTER_FIRE_TIME_MS / 1000.0:
                self.target_position = self._next_prime_position()
                self.autofire_state = AutofireState.WAITFORBLOCK
