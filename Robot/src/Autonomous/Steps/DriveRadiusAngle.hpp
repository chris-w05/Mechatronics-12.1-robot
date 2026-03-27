/**
 * @file DriveRadiusAngle.hpp
 * @brief `AutoStep` that drives along an arc for a specified heading change (degrees).
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Follow a constant-radius arc until the accumulated heading changes by `angleToTurnDeg`.
 *
 * A single autonomous step for driving a certain number of degrees along an arc
 * at a specified velocity and radius.
 */
class DriveRadiusAngle : public AutoStep
{
public:
        /**
         * @brief Full constructor.
         * @param drive           Drive subsystem reference.
         * @param targetVelocity  Arc speed (m/s).
         * @param radius          Arc radius (m).
         * @param angleToTurnDeg  Heading change to achieve (degrees).
         */
        DriveRadiusAngle(Drive &drive,
                         float targetVelocity,
                         float radius,
                         float angleToTurnDeg)
        : _drive(drive),
          _velocity(targetVelocity),
          _radius(radius),
          _target_angle(angleToTurnDeg * PI/180.0) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
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

    /**
     * @brief Re-configure the step for reuse.
     * @param target_distance  Target angle in the units used internally (rad after conversion).
     * @param target_velocity  Arc speed (m/s).
     * @param radius           Arc radius (m).
     */
    void configure(float target_distance, float target_velocity, float radius)
    {
        _target_angle = target_distance;
        _velocity = target_velocity;
        _radius = radius;
    }

private:
    Drive &_drive;          ///< Drive subsystem reference
    float _velocity = 0;    ///< Arc speed (m/s)
    float _radius = 10;     ///< Arc radius (m)
    float _target_angle = 0; ///< Heading change to achieve (radians)
    float _startAngle = 0;  ///< Accumulated heading at step start (rad)
};