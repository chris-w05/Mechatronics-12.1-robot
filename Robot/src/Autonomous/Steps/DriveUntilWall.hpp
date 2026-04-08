/**
 * @file DriveUntilWall.hpp
 * @brief `AutoStep` that drives straight toward a wall until the distance sensor reaches a setpoint.
 *
 * Uses `Drive::approachDistance()` for closed-loop wall approach.
 * Finishes when the sensor reads within 0.5 cm of `_target` AND the robot velocity is < 1 cm/s.
 */
#pragma once
#ifndef DRIVE_UNTIL_WALL_STEP_H
#define DRIVE_UNTIL_WALL_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Approach a wall using the front distance sensor until within 0.5 cm of `_target`.
 */
class DriveToWallStep : public AutoStep
{
public:
    /**
     * @brief Full constructor.
     * @param drive          Drive subsystem reference.
     * @param targetDistance Target wall distance (cm).
     */
    DriveToWallStep(Drive &drive,
                    float targetDistance)
        : _drive(drive),
          _target(targetDistance) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
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
        return ( abs(_drive.getDistanceSensorReading() - _target) < .5 ) && abs( _drive.getAvgVelocity() ) < 1 ;
    }

    void end()
    {
        _drive.setSpeed(0);
    }

    /**
     * @brief Re-configure for reuse.
     * @param targetDistance  Wall stand-off distance (cm).
     * @param targetVelocity  Approach speed (unused; handled by Drive internally).
     */
    void configure(float targetDistance, float targetVelocity)
    {
        _target = targetDistance;
        _velocity = targetVelocity;
    }

private:
    Drive &_drive;            ///< Drive subsystem reference
    float _target = 5;        ///< Wall stand-off distance (cm)
    float _velocity = 0;      ///< Approach speed parameter (currently unused)
    float _startDistance = 0; ///< Odometry snapshot at step start (unused)
};

#endif
