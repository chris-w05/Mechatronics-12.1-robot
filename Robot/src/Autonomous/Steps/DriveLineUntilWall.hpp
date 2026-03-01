#pragma once
#ifndef DRIVE_LINE_UNTIL_WALL_STEP_H
#define DRIVE_LINE_UNTIL_WALL_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class DriveLineToWallStep : public AutoStep
{
public:
    DriveLineToWallStep(Drive &drive,
                    float targetDistance,
                    float targetVelocity)
        : _drive(drive),
          _target(targetDistance),
          _velocity(targetVelocity) {}

    DriveLineToWallStep(Drive &drive)
        : _drive(drive) {}

    void start() override
    {
        _drive.followLineHardset(_velocity);
    }

    void update() override
    {
        // float error = _drive.getDistanceSensorReading() - _target;
        // if (abs(error) < 8  ) //Only applies if the robot is 8cm within target
        // {
        //     //Change the target speed to make the robot stop. 
        //     _velocity = 2 * error;
        //     _drive.followLine(_velocity);
        // }
    }

    bool isFinished() const override
    {
        // Within 1cm of target, and travelling at less than 1 in/s
        float distance = _drive.getDistanceSensorReading();
        Serial.print(">Distance:");
        Serial.print(distance);
        Serial.println("\r");
        return ( abs( distance - _target) < 1 ) ;
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