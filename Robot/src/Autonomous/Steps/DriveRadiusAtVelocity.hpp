#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class DriveRadiusAtVelocity : public AutoStep
{
public:
   
    DriveRadiusAtVelocity(Drive &drive,
             float targetVelocity,
             float radius,
             float distance)
        : _drive(drive),
          _velocity(targetVelocity),
          _radius(radius),
          _target_distance(distance) {}

    DriveRadiusAtVelocity(Drive &drive)
        : _drive(drive) {}

    DriveRadiusAtVelocity();

    void start() override
    {
        _startDistance = _drive.getDistance();
        _drive.followRadiusAtVelocity(_velocity, _radius);
    }

    void update() override
    {
    }

    bool isFinished() const override
    {
        // Serial.print(_drive.getAccumulatedHeading() - _startAngle);
        // Serial.print("  ");
        // Serial.println(_target);
        // Serial.print("Drive Arc, Delta distance ");
        // Serial.println((_drive.getDistance() - _startDistance));
        return abs(_drive.getDistance() - _startDistance) >= abs(_target_distance);
    }

    void end()
    {
        Serial.print("heading: ");
        Serial.println(_drive.getPose().heading);
        _drive.hardSetSpeed(0);
    }

    void configure(float target_distance, float target_velocity, float radius)
    {
        _target_distance = target_distance;
        _velocity = target_velocity;
        _radius = radius;
    }

private:
    Drive &_drive;
    float _velocity = 0;
    float _radius = 10;
    float _target_distance = 0;
    float _startDistance = 0;
};