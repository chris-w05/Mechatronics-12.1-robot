/**
 * @file FollowLineStep.hpp
 * @brief `AutoStep` that follows a line for a specified distance.
 *
 * Uses `Drive::followLineHardset()` to engage the line-following controller
 * at a fixed speed, finishing when odometry reports the target distance covered.
 */
#pragma once
#ifndef FOLLOW_LINE_STEP_H
#define FOLLOW_LINE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Follow the floor line at a fixed velocity for `_target` meters.
 */
class FollowLineStep : public AutoStep
{
public:
    /**
     * @brief Full constructor.
     * @param drive          Drive subsystem reference.
     * @param targetDistance Distance to follow the line (m).
     * @param targetVelocity Line-follow speed (m/s).
     */
    FollowLineStep(Drive &drive,
                  float targetDistance,
                  float targetVelocity)
        : _drive(drive),
          _target(targetDistance),
          _velocity(targetVelocity) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
    FollowLineStep(Drive &drive)
        : _drive(drive) {}

    void start() override
    {
        _startDistance = _drive.getDistance(); // or encoder average
        _drive.followLineHardset(_velocity);
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

    /**
     * @brief Re-configure for reuse.
     * @param targetDistance  Distance to follow (m).
     * @param targetVelocity  Speed (m/s).
     */
    void configure(float targetDistance, float targetVelocity)
    {
        _target = targetDistance;
        _velocity = targetVelocity;
    }

private:
    Drive &_drive;            ///< Drive subsystem reference
    float _target = 0;        ///< Distance to follow (m)
    float _velocity = 0;      ///< Line-follow speed (m/s)
    float _startDistance = 0; ///< Odometry distance at step start (m)
};

#endif