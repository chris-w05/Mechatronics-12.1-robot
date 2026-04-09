/**
 * @file DriveRadiusAngle.hpp
 * @brief `AutoStep` that drives along an arc for a specified heading change (degrees).
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Follow a constant-radius arc until the drivetrain sees a line.
 *
 * A single autonomous step for driving a certain number of degrees along an arc
 * at a specified velocity and radius.
 */
class DriveRadiusToLine : public AutoStep
{
public:
        /**
         * @brief Full constructor.
         * @param drive           Drive subsystem reference.
         * @param targetVelocity  Arc speed (in/s).
         * @param radius          Arc radius (in).
         */
        DriveRadiusToLine(Drive &drive,
                         float targetVelocity,
                         float radius)
        : _drive(drive),
          _velocity(targetVelocity),
          _radius(radius){}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
    DriveRadiusToLine(Drive &drive)
        : _drive(drive) {}

    DriveRadiusToLine();

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
        return _drive.isCenteredOverLine(); 
    }

    void end()
    {
        _drive.setSpeed(0);
    }

    /**
     * @brief Re-configure the step for reuse.
     * @param target_velocity  Arc speed (m/s).
     * @param radius           Arc radius (m).
     */
    void configure( float target_velocity, float radius)
    {
        _velocity = target_velocity;
        _radius = radius;
    }

private:
    Drive &_drive;          ///< Drive subsystem reference
    float _velocity = 0;    ///< Arc speed (m/s)
    float _radius = 10;     ///< Arc radius (m)
    float _startAngle = 0;  ///< Accumulated heading at step start (rad)
};