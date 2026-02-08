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
    const int distPin)
    : _leftEncoder(left_enc_a, left_enc_b),
      _rightEncoder(right_enc_a, right_enc_b),
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
}

void Drive::update()
{
    //Update child sensors
    _leftEncoder.update();
    _rightEncoder.update();

    _motorController.setTarget(_speedL, _speedR);

    _motorController.update(_leftEncoder.getVelocity(),
                            _rightEncoder.getVelocity());

    _odometry.update(_leftEncoder, _rightEncoder);
}

Odometry::Pose2D Drive::getPose(){
    return _odometry.getPose();
}

void Drive::setSpeed(int16_t speed){
    _speedR = speed;
    _speedL = speed;
}

void Drive::followRadiusClockwise(int16_t speed, float radius )
{
    _speedL = speed * (radius + DRIVETRAIN_WIDTH/2 ) / radius;
    _speedR = speed * (radius - DRIVETRAIN_WIDTH/2 ) / radius;
}

void Drive::followRadiusCCW(int16_t speed, float radius)
{
    _speedL = speed * (radius - DRIVETRAIN_WIDTH / 2) / radius;
    _speedR = speed * (radius + DRIVETRAIN_WIDTH / 2) / radius;
}

void Drive::stop()
{
    setSpeed(0);
}
