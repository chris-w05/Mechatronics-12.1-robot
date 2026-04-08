/**
 * @file DriveRadiusAtVelocity.hpp
 * @brief `AutoStep` that drives along a constant-radius arc for a specified arc length.
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Follow a constant-radius arc until the odometry distance increases by `_target_distance`.
 *
 * A single autonomous step for driving a distance along an arc at a specified
 * velocity and radius.
 */
class DriveRadiusAtVelocity : public AutoStep
{
public:
        /**
         * @brief Full constructor.
         * @param drive          Drive subsystem reference.
         * @param targetVelocity Arc speed (in/s).
         * @param radius         Arc radius (in).
         * @param distance       Arc length to travel (in).
         */
        DriveRadiusAtVelocity(Drive &drive,
                         float targetVelocity,
                         float radius,
                         float distance)
        : _drive(drive),
          _velocity(targetVelocity),
          _radius(radius),
          _target_distance(distance) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
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
        _drive.setSpeed(0);
    }

    /**
     * @brief Re-configure for reuse.
     * @param target_distance  Arc length to travel (m).
     * @param target_velocity  Arc speed (m/s).
     * @param radius           Arc radius (m).
     */
    void configure(float target_distance, float target_velocity, float radius)
    {
        _target_distance = target_distance;
        _velocity = target_velocity;
        _radius = radius;
    }

private:
    Drive &_drive;              ///< Drive subsystem reference
    float _velocity = 0;        ///< Arc speed (m/s)
    float _radius = 10;         ///< Arc radius (m)
    float _target_distance = 0; ///< Arc length to travel (m)
    float _startDistance = 0;   ///< Odometry distance at step start (m)
};