/**
 * @file DriveDistance.hpp
 * @brief `AutoStep` that drives straight for a specified distance at a fixed velocity.
 */
#ifndef DRIVEDISTANCE_STEP_H
#define DRIVEDISTANCE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Drive in a straight line for `targetDistance` meters at `targetVelocity` m/s.
 */
class DriveDistance : public AutoStep{
public:
    /**
     * @brief Full constructor.
     * @param drive          Drive subsystem reference.
     * @param targetDistance Distance to travel (m, sign sets direction).
     * @param targetVelocity Speed to drive at (m/s, sign sets direction).
     */
    DriveDistance(Drive &drive,
                float targetDistance,
                float targetVelocity)
        : _drive(drive),
          _target(targetDistance),
          _velocity(targetVelocity) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
    DriveDistance(Drive &drive)
        : _drive(drive){}

    void start() override
    {
        
        _startDistance = _drive.getDistance(); // or encoder average
        _drive.setSpeed(_velocity);
    }

    void update() override
    {
    }

    bool isFinished() const override
    {
        return abs(_drive.getDistance() - _startDistance) >= abs(_target);
    }
    

    void end(){
        _drive.setSpeed(0);
    }

    /**
     * @brief Re-configure for reuse.
     * @param targetDistance  Distance to travel (m).
     * @param targetVelocity  Speed (m/s).
     */
    void configure(float targetDistance, float targetVelocity){
        _target = targetDistance;
        _velocity = targetVelocity;

    }

private:
    Drive &_drive;             ///< Drive subsystem reference
    float _target = 0;         ///< Target distance (m)
    float _velocity = 0;       ///< Drive speed (m/s)
    float _startDistance = 0;  ///< Odometry distance at step start (m)
};


#endif