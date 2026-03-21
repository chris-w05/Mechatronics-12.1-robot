#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * A single autonomous step for driving a certain number of degrees along an arc at a specified velocity and radius
 */
class DriveRadiusAngle : public AutoStep
{
public:
   
    DriveRadiusAngle(Drive &drive,
             float targetVelocity,
             float radius,
             float angleToTurnDeg)
        : _drive(drive),
          _velocity(targetVelocity),
          _radius(radius),
          _target_angle(angleToTurnDeg * PI/180.0) {}

    DriveRadiusAngle(Drive &drive)
        : _drive(drive) {}

    DriveRadiusAngle();

    void start() override
    {
        _startAngle = _drive.getAccumulatedHeading();
        _drive.followRadiusAtVelocity(_velocity, _radius);
    }

    void update() override
    {
    }

    bool isFinished() const override
    {
        return abs(_drive.getAccumulatedHeading() - _startAngle) >= abs(_target_angle);
    }

    void end()
    {
        _drive.setSpeed(0);
    }

    void configure(float target_distance, float target_velocity, float radius)
    {
        _target_angle = target_distance;
        _velocity = target_velocity;
        _radius = radius;
    }

private:
    Drive &_drive;
    float _velocity = 0;
    float _radius = 10;
    float _target_angle = 0;
    float _startAngle = 0;
};