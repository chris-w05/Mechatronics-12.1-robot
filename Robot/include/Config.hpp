#pragma once

#include <Arduino.h>
#include <Devices/DualMotorController.hpp>
#include "utils/PID.hpp"
#include <cmath>

//----------------------------Pin allocation
// Encoder pins
static const uint16_t LEFT_ENCODER_A = 18;
static const uint16_t LEFT_ENCODER_B = 19;
static const uint16_t RIGHT_ENCODER_A = 20;
static const uint16_t RIGHT_ENCODER_B = 21;

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

static const uint16_t SHOOTER_ENCODER_A  = 3;
static const uint16_t SHOOTER_ENCODER_B = 26;

// Servo pins
static const uint16_t MINER_SERVO_PIN = 11;

//Serial pins
static const uint16_t SERIAL_RX = 16;
static const uint16_t SERIAL_TX = 17;
static const long SERIAL_BAUD_RATE = 115200;

//Sensor pins
static const uint16_t HALL_EFFECT_PIN = -1;
static const uint16_t LINE_SENSOR_PINS[8] = {28, 30, 32, 34, 36, 38, 40, 42}; 
static const uint16_t COLOR_SENSOR_START_PIN = -1; // CHANGE ME
static const uint16_t DISTANCE_SENSOR_PIN = A4; // CHANGE ME

static const uint16_t SHOOTER_LIMIT_PIN = 27;

//----------------------------------Software constants
static const int MAX_STEPS = 10; // Maximum allowed steps in autonomous. This prevents too much memory from being used
static const int MAX_SUBSYSTEMS = 10; 

//----------------------------------Physical constants
//Robot params

static const int TICKS_PER_REV = 64; //Number of ticks per motor revolution
static const float DRIVETRAIN_WIDTH = 9.375; // inches
static const float DRIVETRAIN_MOTOR_RATIO = 50 * 45 / 100; // motor ratio * wheel teeth * motor teeth
static const float DRIVETRAIN_WHEEL_DIAMETER = 3.93700787402; // in
static const float DRIVETRAIN_TICKS_TO_IN = PI * DRIVETRAIN_WHEEL_DIAMETER / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV);

static const float LINESENSOR_LOCATION  = 3;//in (from middle of left wheel)
static const float LINESENSOR_LR_RATIO = (DRIVETRAIN_WIDTH - LINESENSOR_LOCATION) - LINESENSOR_LOCATION; //Handles the difference in kp required for left and right side of the robot for line following
const uint16_t lineSensorCalMin[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const uint16_t lineSensorCalMax[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

static const float SHOOTER_MOTOR_RATIO = 70;
static const float SHOOTER_TICKS_TO_ROTATIONS = 1.0/ (SHOOTER_MOTOR_RATIO * TICKS_PER_REV);


static const float IR_SENSOR_TO_ROBOT_EDGE = 1.5; //cm
static float distanceSensor_VoltageToDistance(float voltage){
    return pow((voltage / 3.02398943), -1.43921480) - IR_SENSOR_TO_ROBOT_EDGE;
}
//----------------------------------Setpoints


//Miner constants
static const unsigned long MINER_CYCLE_MS = 400UL;                  //ms
static const unsigned long MINER_PRESS_MS = 100UL;
static const int MINER_SERVO_PRESS_ANGLE = 140;
static const int MINER_SERVO_STORE_ANGLE = 00;
static const int MINER_SERVO_RETRACT_ANGLE = 180;

static const float SHOOTER_PULL_BACK_ROTATIONS = .75;
static const int SHOOTER_SETTLE_TIME = 200; //Time for block to settle in shooter ms
static const int SHOOTER_FIRE_TIME = 800; //Time after shooter firing before priming again

//----------------------------------PID constants

/**
 * Minimum signal for drivetrain to move
 */
static const int stallSignal = 120;
static const int maxSignal = 400;
static const float maxVelocity = 45.0; //in/s, velocity of the robot when drive motors are set to their maximum power
static const float a = ( maxSignal- stallSignal)/(maxVelocity);

/**
 * FeedForward control for drivetrain - this gives an approximate expectiation of required motor signal for a given velocity
 */
static constexpr float driveFF( float target){
    return 0.0;
    // return target * 10;
    // return a * target + stallSignal;
}


static const float DRIVE_LINEFOLLOW_GAIN = 3.8;
static const float DRIVE_LINEFOLLOW_VELOCITY_GAIN = 6;

static const PIDConstants DRIVE_DISTANCE_PID = {
    .kp = -20.0,
    .ki = 0,
    .kd = 8};

// drivetrain
static const PIDConstants DRIVE_L_PID = {
    .kp = 200.0,
    .ki = 100,
    .kd = -5};

static const PIDConstants DRIVE_R_PID = DRIVE_L_PID;
// {
//     .kp = 30.0,
//     .ki = 2,
//     .kd = .2
// };

static const int SHOOTER_STALL_SIGNAL = 60;
/**
 * FeedForward control for shoooter - overcomes frictional forces
 * */
static float shooterFF(float measurement, float target)
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


static const PIDConstants SHOOTER_POSITION_PID = {
    .kp = 6000.0,
    .ki = 00.0,
    .kd = -1000.0
};

static const PIDConstants SHOOTER_VELOCITY_PID = {
    .kp = 000.0,
    .ki = 000.0,
    .kd = 00.0
};
