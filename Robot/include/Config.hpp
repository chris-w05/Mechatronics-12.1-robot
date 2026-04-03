/**
 * @file Config.hpp
 * @brief Central configuration: pin assignments, physical constants, and PID gains.
 *
 * All hardware pin numbers, drivetrain geometry, miner/shooter tuning values,
 * and PID constants live here so that changes only need to be made in one
 * place. Every subsystem and autonomous step pulls its constants from this
 * file via its includes.
 */
#pragma once

#include <Arduino.h>
#include <Devices/DualMotorController.hpp>
#include "utils/PID.hpp"
#include <cmath>

// ============================================================================
/// @defgroup pins Pin Assignments
/// @{
// ============================================================================

/// @name Drive encoder pins (hardware interrupt-capable)
/// @{
static const uint16_t LEFT_ENCODER_A  = 18; ///< Left  encoder channel A
static const uint16_t LEFT_ENCODER_B  = 19; ///< Left  encoder channel B
static const uint16_t RIGHT_ENCODER_A = 20; ///< Right encoder channel A
static const uint16_t RIGHT_ENCODER_B = 21; ///< Right encoder channel B
/// @}

/// @name Drive motor pin bundle (L298N-compatible via DualMotorController)
/// @{
const TB9051Pins drivePins = {
    //Drive motor pins
    //Left motor
    .m1PWM = 6,
    .m1Direction1 = 31,
    .m1Direction2 = 33,

    //Right motor
    .m2PWM = 7,
    .m2Direction1 = 35,
    .m2Direction2 = 37};
/// @}

/// @name Shooter motor pin bundle
/// M2 pins are unused (set to 255 = NC) because the shooter uses only one motor.
/// @{
const TB9051Pins shooterPins = {
    //Shooter pins
    .m1PWM = 2,
    .m1Direction1 = 39,
    .m1Direction2 = 41,

    //These are unused
    .m2PWM = 255,
    .m2Direction1 = 255,
    .m2Direction2 = 255
};
/// @}

static const uint16_t SHOOTER_ENCODER_A = 3;  ///< Shooter encoder channel A
static const uint16_t SHOOTER_ENCODER_B = 26; ///< Shooter encoder channel B

static const uint16_t MINER_SERVO_PIN = 11; ///< Miner press servo signal pin
static const uint16_t RAMP_SERVO_PIN  = 12; ///< Ramp deployment servo signal pin

/// @name Serial (XBee wireless comms on Serial2)
/// @{
static const uint16_t SERIAL_RX       = 16;      ///< RX2 pin for XBee
static const uint16_t SERIAL_TX       = 17;      ///< TX2 pin for XBee
static const long     SERIAL_BAUD_RATE = 115200; ///< Baud rate for both USB and XBee
/// @}

static const uint16_t HALL_EFFECT_PIN       = -1; ///< Hall-effect sensor pin (unused)
static const uint16_t LINE_SENSOR_PINS[8]   = {46, 44, 43, 45, 47, 49, 51, 53}; ///< QTR sensor pins (left → right)
static const uint16_t COLOR_SENSOR_START_PIN = -1; ///< Color sensor pin (unused — set before use)
static const uint16_t DISTANCE_SENSOR_PIN   = A4; ///< Sharp IR analog pin
static const uint16_t SHOOTER_LIMIT_PIN     = 27; ///< Block-detection limit switch pin (active-LOW via INPUT_PULLUP)

/// @}  // end group pins

// ============================================================================
/// @defgroup sw_constants Software Constants
/// @{
// ============================================================================

static const int MAX_STEPS      = 10; ///< Maximum steps in one AutonomousRoutine (caps stack usage)
static const int MAX_SUBSYSTEMS = 10; ///< Maximum subsystems registered with Robot

/// @}

// ============================================================================
/// @defgroup physical Physical Constants
/// @{
// ============================================================================

static const int   TICKS_PER_REV           = 64;                              ///< Encoder ticks per motor shaft revolution
static const float DRIVETRAIN_WIDTH         = 9.549f;                          ///< Track width, wheel-centre to wheel-centre (inches)
static const float DRIVETRAIN_MOTOR_RATIO   = 50.0f * 30.0f / 45.0f;          ///< Combined gear ratio: gearbox × chain sprockets
static const float DRIVETRAIN_WHEEL_DIAMETER = 100.0f / 25.4f;                ///< Drive wheel diameter (inches, converted from 100 mm)
static const float DRIVETRAIN_TICKS_TO_IN  = PI * DRIVETRAIN_WHEEL_DIAMETER
                                             / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV); ///< Scale factor: encoder ticks → inches of travel

static const float LINESENSOR_LOCATION = 6.0f / 2.54f; ///< Lateral distance of line sensor from left-wheel centre (inches)
/** @brief Left/right PID gain ratio for line following.
 *
 *  Because the sensor is not centred on the robot, the same lateral error
 *  requires different corrections on each side.  This ratio compensates.
 */
static const float LINESENSOR_LR_RATIO = (DRIVETRAIN_WIDTH - LINESENSOR_LOCATION) / LINESENSOR_LOCATION;

const uint16_t LINESENSORCALMIN[8] = {96, 96, 96, 48, 48, 48, 48, 48};     ///< QTR minimum calibration values (per sensor)
const uint16_t LINESENSORCALMAX[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; ///< QTR maximum calibration values

static const float SHOOTER_MOTOR_RATIO       = 70.0f; ///< Shooter gearbox ratio
static const float SHOOTER_TICKS_TO_ROTATIONS = 1.0f / (SHOOTER_MOTOR_RATIO * TICKS_PER_REV); ///< Scale factor: ticks → output shaft rotations

static const float IR_SENSOR_TO_ROBOT_EDGE = 1.5f; ///< Distance from IR sensor window to robot's physical edge (cm)

/**
 * @brief Convert a Sharp IR sensor voltage to distance in centimetres.
 *
 * Fitted power-law calibration curve for the GP2Y0A51 (2–15 cm range).
 * An offset of #IR_SENSOR_TO_ROBOT_EDGE converts sensor-face distance to
 * wall-to-robot-edge distance.
 *
 * @param voltage  Smoothed sensor output voltage (V)
 * @return         Distance from robot edge to wall (cm)
 */
inline float distanceSensor_VoltageToDistance(float voltage){
    return pow((voltage / 3.02398943), -1.43921480) - IR_SENSOR_TO_ROBOT_EDGE;
}

/// @} // end group physical

// ============================================================================
/// @defgroup setpoints Setpoints and Tuning
/// @{
// ============================================================================

/// @name Miner timing and angles
/// @{
static const unsigned long MINER_CYCLE_MS        = 400UL; ///< Full press-retract cycle period (ms)
static const unsigned long MINER_PRESS_MS         = 100UL; ///< Duration the miner servo stays in the pressed position (ms)
static const int           MINER_SERVO_PRESS_ANGLE   = 140; ///< Servo angle while pressing down on the block (°)
static const int           MINER_SERVO_RETRACT_ANGLE = 160; ///< Servo angle after lifting off the block (°)
static const int           MINER_SERVO_STORE_ANGLE   = 180; ///< Servo angle in the stowed/transport position (°)
/// @}

/// @name Shooter tuning
/// @{
static const float SHOOTER_PULL_BACK_ROTATIONS = 0.75f; ///< Output-shaft rotations the rack retracts to when priming
static const int   SHOOTER_SETTLE_TIME         = 200;   ///< Time (ms) to wait for a block to settle before firing
static const int   SHOOTER_FIRE_TIME           = 800;   ///< Time (ms) after firing before priming again
/// @}

/// @name Ramp servo angles
/// @{
static const int RAMP_SERVO_STORE_ANGLE   = 180; ///< Ramp stowed against the robot
static const int RAMP_SERVO_LIFT_ANGLE    = 150; ///< Ramp raised to funnel blocks into the shooter
static const int RAMP_SERVO_PASSIVE_ANGLE = 90;  ///< Ramp in neutral/passive position
/// @}

/// @}  // end group setpoints

// ============================================================================
/// @defgroup pid_constants PID & Motor Constants
/// @{
// ============================================================================

static const int   stallSignal = 120;    ///< Minimum motor signal magnitude needed to overcome static friction
static const int   maxSignal   = 400;    ///< Maximum motor signal (motor driver units, ±400)
static const float MAXVELOCITY = 27.0f; ///< Robot top speed at full motor power (in/s)

static const float DRIVE_LINEFOLLOW_VELOCITY_GAIN = 6.0f; ///< Scales line-position error into a velocity correction (in/s per cm)

/** @brief PID gains for the line-following velocity correction loop. */
static const PIDConstants DRIVE_LINEFOLLOW_GAINS = {
    .kp = 175,
    .ki = 0,
    .kd = 0
};

static const float DRIVE_LINEFOLLOW_GAIN = DRIVE_LINEFOLLOW_GAINS.kp; ///< Convenience alias for the line-follow proportional gain

/** @brief PID gains for the IR-distance-sensor wall-following loop. */
static const PIDConstants DRIVE_DISTANCE_PID = {
    .kp = -200.0,
    .ki = -70,
    .kd = -80};

/** @brief Closed-loop velocity PID gains for the left drive wheel. */
static const PIDConstants DRIVE_L_PID = {
    .kp = 400.0,
    .ki = 100,
    .kd = 8
};

static const PIDConstants DRIVE_R_PID = DRIVE_L_PID; ///< Closed-loop velocity PID gains for the right drive wheel (identical to left)

static const int SHOOTER_STALL_SIGNAL = 60; ///< Minimum shooter motor signal to overcome rubber-band pre-load

/**
 * @brief Nonlinear feedforward for the shooter position controller.
 *
 * Adds a bias signal that overcomes the rubber-band spring force as a
 * function of the rack's target position within one cycle.  The feed
 * component scales with the fractional position so that stall signal is
 * only applied in the direction the rack needs to move.
 *
 * @param measurement  Current encoder position (output-shaft rotations)
 * @param target       Target encoder position (output-shaft rotations)
 * @return             Feedforward motor signal to add to the PID output
 */
inline float shooterFF(float measurement, float target)
{
    float feed = fmod(target, 1.0) * 600;
    float diff = target - measurement;
    if( diff > 0.01){
        return SHOOTER_STALL_SIGNAL + feed;
    }
    else if (diff < -.01){
        return -SHOOTER_STALL_SIGNAL + feed;
    }
    return feed;
}

/** @brief Position (rack-displacement) PID gains for the shooter motor controller. */
static const PIDConstants SHOOTER_POSITION_PID = {
    .kp = 6000.0,
    .ki = 00.0,
    .kd = 1000.0
};

/** @brief Velocity PID gains for the shooter motor (currently zeroed / not in use). */
static const PIDConstants SHOOTER_VELOCITY_PID = {
    .kp = 000.0,
    .ki = 000.0,
    .kd = 00.0
};

/// @} // end group pid_constants
