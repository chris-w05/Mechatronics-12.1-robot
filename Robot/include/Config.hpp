#pragma once

#include <Arduino.h>
#include <Devices/DualMotorController.hpp>
#include "utils/PID.hpp"

//----------------------------Pin allocation
// Encoder pins
static const int LEFT_ENCODER_A = 18;
static const int LEFT_ENCODER_B = 19;
static const int RIGHT_ENCODER_A = 20;
static const int RIGHT_ENCODER_B = 21;

// Motor pins

const TB9051Pins drivePins = {
    //Drive motor pins
    .m1PWM = 6,
    .m1Direction1 = 31,
    .m1Direction2 = 33,
    .m2PWM = 7,
    .m2Direction1 = 35,
    .m2Direction2 = 37};

const TB9051Pins shooterPins = {
    //Shooter pins
    .m1PWM = 2,
    .m1Direction1 = 39,
    .m1Direction2 = 41,
    .m2PWM = 255,
    .m2Direction1 = 255,
    .m2Direction2 = 255
};

static const int SHOOTER_ENCODER_A  = 3;
static const int SHOOTER_ENCODER_B = 26;

// Servo pins
static const int MINER_SERVO_PIN = 11;

//Serial pins
static const int SERIAL_RX = 16;
static const int SERIAL_TX = 17;
static const long SERIAL_BAUD_RATE = 115200;

//Sensor pins
static const int HALL_EFFECT_PIN = A2;
static const int LINE_SENSOR_START_PIN = -1; //CHANGE ME
static const int COLOR_SENSOR_START_PIN = -1; // CHANGE ME
static const int DISTANCE_SENSOR_PIN = -1; // CHANGE ME

//----------------------------------Software constants
static const int MAX_STEPS = 10; // Maximum allowed steps in autonomous. This prevents too much memory from being used
static const int MAX_SUBSYSTEMS = 10; 

//----------------------------------Physical constants
//Robot params
static const float DRIVETRAIN_WIDTH = 9.375; //inches
static const int TICKS_PER_REV = 64; //Number of ticks per motor revolution
static const float DRIVETRAIN_MOTOR_RATIO = 50;
static const float SHOOTER_MOTOR_RATIO = 70;
static const float DRIVETRAIN_WHEEL_DIAMETER = 3.93700787402; // in
static const float DRIVETRAIN_TICKS_TO_IN = PI * DRIVETRAIN_WHEEL_DIAMETER / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV);
static const float ARM_ROT_RATIO = .5; // rev/rotation - arm pos(rotations) = motors rotations * ARM_ROT_RATIO
static const float ARM_EXT_RATIO = 3.14 * 2; // mm/rotation - arm pos(mm) = motors rotations * ARM_EXT_RATIO

//----------------------------------Setpoints


//Miner constants
static const unsigned long MINER_CYCLE_MS = 400UL;                  //ms
static const unsigned long MINER_PRESS_MS = 100UL;
static const int MINER_SERVO_PRESS_ANGLE = 140;
static const int MINER_SERVO_STORE_ANGLE = 00;
static const int MINER_SERVO_RETRACT_ANGLE = 180;


//----------------------------------PID constants

/**
 * Minimum signal for drivetrain to move
 */
static const int stallSignal = 120;
static const int maxSignal = 120;
static const float a = ( maxSignal- stallSignal)/(21.0);

/**
 * FeedForward control for drivetrain - this gives an approximate expectiation of required motor signal for a given velocity
 */
static constexpr float driveFF(float setpoint){
    // return 0;
    return setpoint == 0 ? 0 : a*setpoint + (abs(setpoint)/setpoint)* stallSignal;
}

static const float DRIVE_LINEFOLLOW_GAIN = 40;
static const float DRIVE_LINEFOLLOW_VELOCITY_GAIN = 1;

// drivetrain
static const PIDConstants DRIVE_L_PID = {
    .kp = 20.0,
    .ki = 0,
    .kd = 0};

static const PIDConstants DRIVE_R_PID = DRIVE_L_PID;
// {
//     .kp = 30.0,
//     .ki = 2,
//     .kd = .2
// };

static const PIDConstants SHOOTER_POSITION_PID = {
    .kp = -600.0,
    .ki = 0.0,
    .kd = 10.0
};

static const PIDConstants SHOOTER_VELOCITY_PID = {
    .kp = 120.0,
    .ki = 0.0,
    .kd = 0.0
};
