/**
 * @file DriveLineUntilWall.hpp
 * @brief `AutoStep` that follows a line until the distance sensor reads a target wall distance.
 *
 * Uses `Drive::approachAlongLine()` to combine line-following with distance-sensor feedback.
 * Finishes when the front distance sensor is within 0.5 cm of `targetDistance`.
 */
#pragma once
#ifndef DRIVE_LINE_UNTIL_WALL_STEP_H
#define DRIVE_LINE_UNTIL_WALL_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Follow a line autonomously until the distance sensor indicates a wall at `_target` cm.
 */
class DriveLineToWallStep : public AutoStep
{
public:
    /**
     * @brief Full constructor.
     * @param drive          Drive subsystem reference.
     * @param targetDistance Target wall distance in cm.
     */
    DriveLineToWallStep(Drive &drive,
                    float targetDistance)
        : _drive(drive),
          _target(targetDistance) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
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

        //I added this so the robot continues to drive into the wall
        _drive.pulse(160, 1000000, 3000000);
    }

    /**
     * @brief Re-configure the target wall distance.
     * @param targetDistance  Target distance in cm.
     */
    void configure(float targetDistance)
    {
        _target = targetDistance;
    }

private:
    Drive &_drive;            ///< Drive subsystem reference
    float _target = 5;        ///< Wall distance to approach (cm)
    float _startDistance = 0; ///< Odometry snapshot at step start (unused)
};

#endif
