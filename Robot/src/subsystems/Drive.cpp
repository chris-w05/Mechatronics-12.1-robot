#include <Arduino.h>
#include "Drive.hpp"
#include "Config.hpp"
#include "Devices/EncoderWrapper.hpp"

Drive::Drive(
    const int left_enc_a,
    const int left_enc_b,
    const int right_enc_a,
    const int right_enc_b,
    const TB9051Pins pins,
    const int distPin,
    const int lineFollowerPin)
    : _leftEncoder(left_enc_a, left_enc_b),
      _rightEncoder(right_enc_a, right_enc_b),
      _lineSensor(lineFollowerPin),
      _motorController(
          pins,
          DRIVE_L_KP, DRIVE_L_KI, DRIVE_L_KD, true,
          DRIVE_R_KP, DRIVE_R_KI, DRIVE_R_KD, true,
          false, false),
      distSensor( distPin)
{
}

void Drive::init()
{
    _odometry.init({0, 0, 0});
    _leftEncoder.init();
    _rightEncoder.init();
    _leftEncoder.flipDirection();
    _motorController.init();
    Serial.println("Drivetrain initialized");
}

void Drive::update()
{
    //Update child sensors

    
    _leftEncoder.update();
    _rightEncoder.update();

    if (mode == STRAIGHT || mode == ARC){
        //Any special considerations handled here
    }
    else if( mode == LINEFOLLOWING){
        _lineSensor.update();
        int correction = _lineSensor.readValue();
        correction *= 20; // Correction gain - velocity units/number sensors active
        if( (_speedL + _speedR)/2 < 0) correction *= -1; //If driving backwards, the line following correction needs to be reversed
        _speedL += correction;
        _speedR -= correction;
    }
    else if (mode == STOPPED){

    }
    _motorController.setTarget(_speedL, _speedR);

    float leftVelocity = _leftEncoder.getVelocity() * PI * DRIVETRAIN_WHEEL_DIAMETER  / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV); //inch/s
    float rightVelocity = _rightEncoder.getVelocity() * PI * DRIVETRAIN_WHEEL_DIAMETER / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV); //inch/s
    Serial.print("Velocity L: ");
    Serial.print(_leftEncoder.getVelocity());
    Serial.print(" targetL: ");
    Serial.print(_speedL);
    Serial.print(" leftVel(inch/s): ");
    Serial.print(leftVelocity);

    Serial.print(" Velocity R: ");
    Serial.print(_rightEncoder.getVelocity());
    Serial.print(" targetR: ");
    Serial.print(_speedR);
    Serial.print(" rightVel(inch/s): ");
    Serial.println(rightVelocity);
    // Serial.print("Mode is ");
    // Serial.println(mode);

    if( mode != HARDSET){
        _motorController.update( leftVelocity, rightVelocity);
    }
    else{
        _motorController.setPower((int)_speedL, -(int)_speedR);
        // Serial.print("Setting motor power to ");
        // Serial.print(_speedL);
        // Serial.print(" ");
        // Serial.println(_speedR);
    }

    _odometry.update(_leftEncoder, _rightEncoder);
    }

Odometry::Pose2D Drive::getPose(){
    return _odometry.getPose();
}



void Drive::setSpeed(float speed){
    mode = STRAIGHT;
    _speedR = speed;
    _speedL = speed;
}

void Drive::hardSetSpeed(int16_t speed){
    mode = HARDSET;
    _speedR = speed;
    _speedL = speed;
    _motorController.setPower(speed, speed);
}

void Drive::hardSetSpeed(int16_t speed1, int16_t speed2)
{
    mode = HARDSET;
    _speedR = speed1;
    _speedL = speed2;
    _motorController.setPower(speed1, speed2);
}

void Drive::followRadiusClockwise(float omega_rad_s, float radius_m)
{
    mode = MODE::ARC;

    // track half-width
    const float halfL = DRIVETRAIN_WIDTH / 2.0f;

    _speedL = omega_rad_s * (radius_m + halfL);
    _speedR = omega_rad_s * (radius_m - halfL);
}

void Drive::followLine(float speed)
{
    mode = MODE::LINEFOLLOWING;
    _speedL = speed;
    _speedR = speed;
}

void Drive::followRadiusCCW(float omega_rad_s, float radius_m)
{
    mode = MODE::ARC;

    // track half-width
    const float halfL = DRIVETRAIN_WIDTH / 2.0f;

    _speedL = omega_rad_s * (radius_m - halfL);
    _speedR = omega_rad_s * (radius_m + halfL);
}

void Drive::stop()
{
    mode = MODE::STOPPED;
    setSpeed(0);
}
