#pragma once
#ifndef DRIVE_UNTIL_WALL_STEP_H
#define DRIVE_UNTIL_WALL_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class DriveToWallStep : public AutoStep
{
public:
    DriveToWallStep(Drive &drive,
                    float targetDistance)
        : _drive(drive),
          _target(targetDistance) {}

    DriveToWallStep(Drive &drive)
        : _drive(drive) {}

    void start() override
    {
        _drive.approachDistance(_target);
    }

    void update() override
    {

    }

    bool isFinished() const override
    {
        //Within 1cm of target, and travelling at less than 1 in/s
        return ( abs(_drive.getDistanceSensorReading() - _target) < 1 ) && abs( _drive.getAvgVelocity() ) < 1 ;
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
    float _target = 5;
    float _velocity = 0;
    float _startDistance = 0;
};

#endif