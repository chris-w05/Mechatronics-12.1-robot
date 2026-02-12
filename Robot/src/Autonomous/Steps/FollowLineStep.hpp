#pragma once
#ifndef FOLLOW_LINE_STEP_H
#define FOLLOW_LINE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class FollowLineStep : public AutoStep
{
public:
    FollowLineStep(Drive &drive,
                  float targetDistance,
                  float targetVelocity)
        : _drive(drive),
          _target(targetDistance),
          _velocity(targetVelocity) {}

    FollowLineStep(Drive &drive)
        : _drive(drive) {}

    void start() override
    {
        _startDistance = _drive.getDistance(); // or encoder average
        _drive.followLine(_velocity);
    }

    void update() override
    {
    }

    bool isFinished() const override
    {
        return (_drive.getDistance() - _startDistance) >= _target;
    }

    void end()
    {
        _drive.setSpeed(0);
    }

    void configure(float targetDistance, float targetVelocity)
    {
        _target = targetDistance;
        _velocity = targetVelocity;
    }

private:
    Drive &_drive;
    float _target = 0;
    float _velocity = 0;
    float _startDistance = 0;
};

#endif