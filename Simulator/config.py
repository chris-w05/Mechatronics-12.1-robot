"""
config.py  –  Mirror of Robot/include/Config.hpp

All physical constants, PID gains, pin-equivalent identifiers, and setpoints
used by the simulator.  Values are taken directly from the C++ source.
"""

import math

# ─── Physical constants ──────────────────────────────────────────────────────

ROBOT_MASS_KG = 3.26

TICKS_PER_REV = 64
DRIVETRAIN_WIDTH = 9.549           # inches  (wheel-centre to wheel-centre)
DRIVETRAIN_MOTOR_RATIO = 50.0 * 30.0 / 45.0   # ≈ 33.333
DRIVETRAIN_WHEEL_DIAMETER = 100.0 / 25.4       # ≈ 3.937 inches
DRIVETRAIN_TICKS_TO_IN = (
    math.pi * DRIVETRAIN_WHEEL_DIAMETER / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV)
)

LINESENSOR_LOCATION = 3.0         # inches from middle of left wheel
LINESENSOR_LR_RATIO = (DRIVETRAIN_WIDTH - LINESENSOR_LOCATION) / LINESENSOR_LOCATION

SHOOTER_MOTOR_RATIO = 70
SHOOTER_TICKS_TO_ROTATIONS = 1.0 / (SHOOTER_MOTOR_RATIO * TICKS_PER_REV)

IR_SENSOR_TO_ROBOT_EDGE = 1.5     # cm

# ─── Motor specs  — Pololu #4753 (50:1 HPCB 6V micro metal gearmotor) ───────
MOTOR_STALL_TORQUE_OZ_IN = 85.0    # oz·in at 6V
MOTOR_FREE_SPEED_RPM = 200.0       # RPM at 6V no-load
MOTOR_STALL_CURRENT_A = 1.6        # A at 6V
MOTOR_FREE_CURRENT_A = 0.07        # A at 6V no-load
MOTOR_NOMINAL_VOLTAGE = 6.0        # V

# Battery / electrical
BATTERY_VOLTAGE = 10.6             # V  (user-specified operating voltage)
# Scale motor performance linearly with voltage ratio
VOLTAGE_RATIO = BATTERY_VOLTAGE / MOTOR_NOMINAL_VOLTAGE

# ─── Servo specs  — MG995 ────────────────────────────────────────────────────
SERVO_SPEED_DEG_PER_SEC = 300.0    # ~60°/0.2s at 6V → 300°/s
SERVO_TORQUE_KG_CM = 13.0          # typical stall torque

# ─── Setpoints ───────────────────────────────────────────────────────────────

# Miner
MINER_CYCLE_MS = 400
MINER_PRESS_MS = 100
MINER_SERVO_PRESS_ANGLE = 140
MINER_SERVO_RETRACT_ANGLE = 160
MINER_SERVO_STORE_ANGLE = 180

# Shooter
SHOOTER_PULL_BACK_ROTATIONS = 0.75
SHOOTER_SETTLE_TIME_MS = 200
SHOOTER_FIRE_TIME_MS = 800

# Ramp
RAMP_SERVO_STORE_ANGLE = 180
RAMP_SERVO_LIFT_ANGLE = 150
RAMP_SERVO_PASSIVE_ANGLE = 90

# ─── PID constants ───────────────────────────────────────────────────────────

STALL_SIGNAL = 120
MAX_SIGNAL = 400
MAX_VELOCITY = 27.0                # in/s

DRIVE_L_PID = {"kp": 400.0, "ki": 100.0, "kd": 8.0}
DRIVE_R_PID = {"kp": 400.0, "ki": 100.0, "kd": 8.0}

DRIVE_LINEFOLLOW_GAINS = {"kp": 50.0, "ki": 0.0, "kd": 0.0}
DRIVE_LINEFOLLOW_VELOCITY_GAIN = 6.0

DRIVE_DISTANCE_PID = {"kp": -40.0, "ki": 0.0, "kd": -12.0}

SHOOTER_POSITION_PID = {"kp": 6000.0, "ki": 0.0, "kd": 1000.0}
SHOOTER_STALL_SIGNAL = 60

# ─── Ramsete controller defaults ─────────────────────────────────────────────
RAMSETE_B = 0.03
RAMSETE_ZETA = 0.4

# ─── Initial pose ────────────────────────────────────────────────────────────
INITIAL_X = 2.0     # inches from wall
INITIAL_Y = 6.0     # inches — middle of start corral
INITIAL_HEADING = 0.0  # radians

# ─── Simulation ──────────────────────────────────────────────────────────────
SIM_DT = 0.006       # nominal (average) simulation timestep
SIM_DT_MIN = 0.005   # 5 ms — random loop-time range (low)
SIM_DT_MAX = 0.008   # 8 ms — random loop-time range (high)
SIM_SPEED = 1.0      # 1.0 = real-time

# ─── Sensor noise (Gaussian std-dev) ─────────────────────────────────────────
ENCODER_VEL_NOISE_TICKS_PER_S = 30.0   # ~0.6% of full-speed velocity
ENCODER_COUNT_NOISE_TICKS = 0.4        # sub-tick quantisation noise per step
LINE_SENSOR_NOISE_CM = 0.15            # Sharp line-sensor lateral noise
DISTANCE_SENSOR_NOISE_CM = 0.5        # Sharp GP2Y0A51 reading noise
SHOOTER_POSITION_NOISE_ROT = 0.002    # encoder noise on shooter rotations

# ─── Distance sensor conversion (Sharp GP2Y0A51) ────────────────────────────
def distance_sensor_voltage_to_cm(voltage: float) -> float:
    """Voltage → distance (cm), matching the C++ distanceSensor_VoltageToDistance."""
    if voltage < 0.01:
        return 30.0  # out of range → clamp
    return (voltage / 3.02398943) ** (-1.43921480) - IR_SENSOR_TO_ROBOT_EDGE

# ─── Shooter feedforward ────────────────────────────────────────────────────
def shooter_ff(measurement: float, target: float) -> float:
    """Mimic the C++ shooterFF feedforward for rubber-band compensation."""
    feed = (target % 1.0) * 600.0
    diff = target - measurement
    if diff > 0.01:
        return SHOOTER_STALL_SIGNAL + feed
    elif diff < -0.01:
        return -SHOOTER_STALL_SIGNAL + feed
    return feed

# ─── Strategy / mining chart ────────────────────────────────────────────────
MINING_CHART = [
    [5,  4,  3,  1],   # WOOD
    [10, 5,  3,  2],   # STONE
    [-1, 10, 5,  3],   # IRON
    [-1, -1, 10, 5],   # DIAMOND
]

SILVERFISH_HITS = [10, 7, 4, 1]

MAX_BLOCK_MINEABLE = [1, 2, 3, 3]  # STONE, IRON, DIAMOND, DIAMOND  (by pickaxe level)
