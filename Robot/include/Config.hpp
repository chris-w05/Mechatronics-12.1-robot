#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

//----------------------------Pin allocation
// Encoder pins
static const int LEFT_ENCODER_A = 18;
static const int LEFT_ENCODER_B = 19;
static const int RIGHT_ENCODER_A = 20;
static const int RIGHT_ENCODER_B = 21;

// Motor pins

//THE ROBOT SHOOTS TOWARD THE BACK
//Left Drive Wheel
static const int LEFT_MOTOR_ENABLE = 2;
static const int LEFT_MOTOR_PWM = 9;
static const int LEFT_MOTOR_DIR = 7;

//Right Drive Wheel
static const int RIGHT_MOTOR_ENABLE = 4;
static const int RIGHT_MOTOR_PWM = 10;
static const int RIGHT_MOTOR_DIR = 8;

//Shooter motor
static const int SHOOTER_MOTOR_ENABLE = 8;
static const int SHOOTER_MOTOR_ENABLE2 = 43;
static const int SHOOTER_MOTOR_PWM1 = 3;
static const int SHOOTER_MOTOR_PWM2 = 5;
static const int SHOOTER_MOTOR_DIR = 45;

//Servo pins
static const int MINER_SERVO_PIN = 11;


//Serial pins
static const int SERIAL_RX = 16;
static const int SERIAL_TX = 17;
static const long SERIAL_BAUD_RATE = 115200;

//Sensor pins
static const int HALL_EFFECT_PIN = A2;
static const int LINE_SENSOR_START_PIN = 28; //CHANGE ME
static const int COLOR_SENSOR_START_PIN = 41; // CHANGE ME
static const int DISTANCE_SENSOR_PIN = 47; // CHANGE ME

//----------------------------------Software constants
static const int MAX_STEPS = 10; // Maximum allowed steps in autonomous. This prevents too much memory from being used
static const int MAX_SUBSYSTEMS = 10; 

//----------------------------------Physical constants
//Robot params
static const float DRIVETRAIN_WIDTH = .23876391; //meters
static const int TICKS_PER_REV = 1024; //Number of ticks per motor revolution
static const float DRIVETRAIN_WHEEL_DIAMETER = 4; //in
static const float ARM_ROT_RATIO = .5; // rev/rotation - arm pos(rotations) = motors rotations * ARM_ROT_RATIO
static const float ARM_EXT_RATIO = 3.14 * 2; // mm/rotation - arm pos(mm) = motors rotations * ARM_EXT_RATIO

//----------------------------------Setpoints


//Miner constants
static const unsigned long MINER_CYCLE_MS = 400UL;                  //ms
static const unsigned long MINER_PRESS_MS = 25UL;
static const int MINER_SERVO_PRESS_ANGLE = 80;
static const int MINER_SERVO_STORE_ANGLE = 160;
static const int MINER_SERVO_RETRACT_ANGLE = 40;


//----------------------------------PID constants
//drivetrain
static const float DRIVE_L_KP = .1;
static const float DRIVE_L_KI = 0.01;
static const float DRIVE_L_KD = 0;
static const float DRIVE_R_KP = .1;
static const float DRIVE_R_KI = 0.01;
static const float DRIVE_R_KD = 0;


//arm
static const float ARM_ROT_KP = .1;
static const float ARM_ROT_KI = 0.01;
static const float ARM_ROT_KD = 0;
//extensi
static const float ARM_EXT_KP = .1;
static const float ARM_EXT_KI = 0.01;
static const float ARM_EXT_KD = 0;

#endif
