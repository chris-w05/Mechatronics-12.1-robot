#pragma once
#ifndef DRIVE_LINE_UNTIL_WALL_STEP_H
#define DRIVE_LINE_UNTIL_WALL_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * A single autonomous step for driving along a line using linefollowing up until a wall is found
 */
class DriveLineToWallStep : public AutoStep
{
public:
    DriveLineToWallStep(Drive &drive,
                    float targetDistance)
        : _drive(drive),
          _target(targetDistance) {}

    DriveLineToWallStep(Drive &drive)
        : _drive(drive) {}

    void start() override
    {
        _drive.approachAlongLine(_target);
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
        // Serial.print(">Distance:");
        // Serial.print(distance);
        // Serial.println("\r");
        return ( abs( distance - _target) < .5 ) ;
    }

    void end()
    {
        //Special case where the drive should continue being snapped to the wall, even as other steps are queued
        //Drive doesn't stop control until other commands are sent
    }

    void configure(float targetDistance)
    {
        _target = targetDistance;
    }

private:
    Drive &_drive;
    float _target = 5;
    float _startDistance = 0;
};

#endif