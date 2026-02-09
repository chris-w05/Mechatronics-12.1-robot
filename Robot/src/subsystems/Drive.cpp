#include <Arduino.h>
#include "Drive.hpp"
#include "Config.hpp"
#include "Devices/EncoderWrapper.hpp"

Drive::Drive(
    const int left_enc_a,
    const int left_enc_b,
    const int right_enc_a,
    const int right_enc_b,
    const int left_mtr_pwm,
    const int left_mtr_dir,
    const int right_mtr_pwm,
    const int right_mtr_dir,
    const int distPin,
    const int lineFollowerPin)
    : _leftEncoder(left_enc_a, left_enc_b),
      _rightEncoder(right_enc_a, right_enc_b),
      _lineSensor(lineFollowerPin),
      _motorController(
          DRIVE_L_KP, DRIVE_L_KI, DRIVE_L_KD, false,
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
        _speedL += correction;
        _speedR -= correction;
    }
    else if (mode == STOPPED){

    }
    _motorController.setTarget(_speedL, _speedR);

    float leftVelocity = _leftEncoder.getVelocity() * PI * DRIVETRAIN_WHEEL_DIAMETER  / TICKS_PER_REV; //inch/s
    float rightVelocity = _rightEncoder.getVelocity() * PI * DRIVETRAIN_WHEEL_DIAMETER / TICKS_PER_REV; //inch/s

    _motorController.update( leftVelocity, rightVelocity);

    _odometry.update(_leftEncoder, _rightEncoder);
    }

Odometry::Pose2D Drive::getPose(){
    return _odometry.getPose();
}



void Drive::setSpeed(int16_t speed){
    mode = STRAIGHT;
    _speedR = speed;
    _speedL = speed;
}



void Drive::followRadiusClockwise(int16_t speed, float radius )
{
    mode = MODE::ARC;
    _speedL = speed * (radius + DRIVETRAIN_WIDTH/2 ) / radius;
    _speedR = speed * (radius - DRIVETRAIN_WIDTH/2 ) / radius;
}

void Drive::followLine(int16_t speed)
{
    mode = MODE::LINEFOLLOWING;
    _speedL = speed;
    _speedR = speed;
}

void Drive::followRadiusCCW(int16_t speed, float radius)
{
    mode = MODE::ARC;
    _speedL = speed * (radius - DRIVETRAIN_WIDTH / 2) / radius;
    _speedR = speed * (radius + DRIVETRAIN_WIDTH / 2) / radius;
}

void Drive::stop()
{
    mode = MODE::STOPPED;
    setSpeed(0);
}
