"""
physics.py  –  Low-level physics models

- PID controller
- Motor model (Pololu #4753 with voltage scaling)
- Encoder simulation
- Servo model (MG995)
- Differential-drive kinematics + odometry
- Ramsete trajectory tracker
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from . import config as C


# ═════════════════════════════════════════════════════════════════════════════
#  PID
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class PIDController:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral: float = 0.0
    prev_error: float = 0.0
    output_min: float = -1e9
    output_max: float = 1e9
    ff_func: object = None  # optional feedforward callable(measurement, target) -> float

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update_simple(self, measurement: float, setpoint: float, dt: float = 1.0) -> float:
        error = setpoint - measurement
        self.integral += error * dt
        # anti-windup: clamp integral contribution
        self.integral = max(-1e6, min(1e6, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        ff = 0.0
        if self.ff_func is not None:
            ff = self.ff_func(measurement, setpoint)
        output = ff + self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))

    def update(self, measurement: float, d_measurement: float,
               setpoint: float, dt: float = 0.001) -> float:
        """PID with derivative of measurement provided (matches C++ Robot PID).

        C++ reference:  output = kp*error + ki*integral - kd*dMeasurement
        integral accumulates as  error * dt  (time-correct).
        """
        error = setpoint - measurement
        self.integral += error * dt
        self.integral = max(-1e6, min(1e6, self.integral))
        ff = 0.0
        if self.ff_func is not None:
            ff = self.ff_func(measurement, setpoint)
        output = ff + self.kp * error + self.ki * self.integral - self.kd * d_measurement
        self.prev_error = error
        return output


# ═════════════════════════════════════════════════════════════════════════════
#  Motor model — Pololu #4753 (50:1 HPCB 6V)
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class MotorModel:
    """Signal → velocity model with first-order dynamics.

    Maps a control signal in [-max_signal, max_signal] to an output velocity
    (in caller-defined units) with an exponential time constant.
    For the drivetrain: velocity is wheel linear speed in in/s.
    For the shooter:   velocity is output-shaft rotations/s.
    """
    max_velocity: float = C.MAX_VELOCITY    # velocity at max signal
    max_signal: float = C.MAX_SIGNAL        # max input signal magnitude
    tau: float = 0.03                       # first-order time constant (s)

    # state
    velocity: float = 0.0
    current_draw_a: float = 0.0

    def step(self, signal: float, dt: float = 0.001) -> float:
        """Advance one timestep.  Returns the current output velocity."""
        clamped = max(-self.max_signal, min(self.max_signal, signal))

        # Linear mapping: signal → target velocity
        if self.max_signal > 0:
            target = clamped / self.max_signal * self.max_velocity
        else:
            target = 0.0

        # First-order lag
        if self.tau > 0:
            alpha = 1.0 - math.exp(-dt / self.tau)
        else:
            alpha = 1.0
        self.velocity += alpha * (target - self.velocity)

        # Current-draw estimate
        duty = abs(clamped) / self.max_signal if self.max_signal > 0 else 0.0
        speed_ratio = (
            min(1.0, abs(self.velocity) / self.max_velocity)
            if self.max_velocity > 0 else 0.0
        )
        self.current_draw_a = duty * (
            C.MOTOR_FREE_CURRENT_A
            + (C.MOTOR_STALL_CURRENT_A - C.MOTOR_FREE_CURRENT_A) * (1.0 - speed_ratio)
        ) * C.VOLTAGE_RATIO

        return self.velocity


# ═════════════════════════════════════════════════════════════════════════════
#  Encoder simulation
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class EncoderSim:
    """Simulates an encoder driven by wheel linear velocity (in/s).

    Converts velocity to tick counts using DRIVETRAIN_TICKS_TO_IN.
    """
    count: float = 0.0
    velocity: float = 0.0       # ticks/sec
    _alpha: float = 0.5         # IIR filter alpha

    def update(self, wheel_velocity_in_per_s: float, dt: float):
        """Advance encoder state given wheel linear velocity."""
        ticks_per_sec = wheel_velocity_in_per_s / C.DRIVETRAIN_TICKS_TO_IN
        # Add sub-tick quantisation + electrical noise to count accumulation
        self.count += ticks_per_sec * dt + random.gauss(0.0, C.ENCODER_COUNT_NOISE_TICKS)
        # Add velocity measurement noise before IIR filter
        noisy_ticks_per_sec = ticks_per_sec + random.gauss(0.0, C.ENCODER_VEL_NOISE_TICKS_PER_S)
        self.velocity = (
            self._alpha * noisy_ticks_per_sec
            + (1.0 - self._alpha) * self.velocity
        )

    def get_count(self) -> float:
        return self.count

    def get_velocity(self) -> float:
        return self.velocity


# ═════════════════════════════════════════════════════════════════════════════
#  Servo model — MG995
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class ServoModel:
    """First-order servo model approaching a target angle."""
    min_angle: float = 0.0
    max_angle: float = 180.0
    speed_deg_per_sec: float = C.SERVO_SPEED_DEG_PER_SEC
    current_angle: float = 90.0
    target_angle: float = 90.0

    def set_angle(self, angle: float):
        self.target_angle = max(self.min_angle, min(self.max_angle, angle))

    def update(self, dt: float):
        diff = self.target_angle - self.current_angle
        max_move = self.speed_deg_per_sec * dt
        if abs(diff) <= max_move:
            self.current_angle = self.target_angle
        else:
            self.current_angle += max_move * (1.0 if diff > 0 else -1.0)


# ═════════════════════════════════════════════════════════════════════════════
#  Pose / Odometry
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0


@dataclass
class Odometry:
    """Integrates left/right wheel displacements into an SE(2) pose.
    Mirrors the C++ Odometry class."""
    pose: Pose2D = field(default_factory=lambda: Pose2D(C.INITIAL_X, C.INITIAL_Y, C.INITIAL_HEADING))
    distance_travelled: float = 0.0
    accumulated_heading: float = 0.0
    _prev_left: float = 0.0
    _prev_right: float = 0.0
    _initialized: bool = False

    def init(self, x: float, y: float, heading: float):
        self.pose = Pose2D(x, y, heading)
        self.distance_travelled = 0.0
        self.accumulated_heading = 0.0
        self._initialized = False

    def update_from_deltas(self, d_left: float, d_right: float):
        """Integrate incremental left/right distances (inches)."""
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / C.DRIVETRAIN_WIDTH

        self.pose.x += d_center * math.cos(self.pose.heading + d_theta / 2.0)
        self.pose.y += d_center * math.sin(self.pose.heading + d_theta / 2.0)
        self.pose.heading += d_theta
        # Wrap heading to [-π, π]
        self.pose.heading = math.atan2(math.sin(self.pose.heading),
                                        math.cos(self.pose.heading))
        self.distance_travelled += abs(d_center)
        self.accumulated_heading += d_theta

    def update_from_encoders(self, left_encoder: EncoderSim, right_encoder: EncoderSim):
        left_pos = left_encoder.get_count() * C.DRIVETRAIN_TICKS_TO_IN
        right_pos = right_encoder.get_count() * C.DRIVETRAIN_TICKS_TO_IN
        if not self._initialized:
            self._prev_left = left_pos
            self._prev_right = right_pos
            self._initialized = True
            return
        d_left = left_pos - self._prev_left
        d_right = right_pos - self._prev_right
        self._prev_left = left_pos
        self._prev_right = right_pos
        self.update_from_deltas(d_left, d_right)


# ═════════════════════════════════════════════════════════════════════════════
#  Ramsete Controller
# ═════════════════════════════════════════════════════════════════════════════

def _sinc(x: float) -> float:
    return 1.0 - x * x / 6.0 if abs(x) < 1e-4 else math.sin(x) / x

def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


@dataclass
class RamseteController:
    b: float = C.RAMSETE_B
    zeta: float = C.RAMSETE_ZETA
    enabled: bool = True

    def step(self, ref: Pose2D, v_ref: float, w_ref: float, cur: Pose2D):
        """Returns (v_cmd, w_cmd)."""
        if not self.enabled:
            return v_ref, w_ref

        dx = ref.x - cur.x
        dy = ref.y - cur.y
        c = math.cos(cur.heading)
        s = math.sin(cur.heading)

        ex = c * dx + s * dy
        ey = -s * dx + c * dy
        etheta = _wrap_pi(ref.heading - cur.heading)

        k = 2.0 * self.zeta * math.sqrt(w_ref ** 2 + self.b * v_ref ** 2)

        v_cmd = v_ref * math.cos(etheta) + k * ex
        w_cmd = w_ref + k * etheta + self.b * v_ref * _sinc(etheta) * ey
        return v_cmd, w_cmd
