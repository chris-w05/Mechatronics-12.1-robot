/**
 * @file AccelerateStraightLine.hpp
 * @brief `AutoStep` that drives straight while ramping speed up to a target velocity.
 *
 * Accelerates from zero at the given rate until the acceleration distance is
 * reached, then holds `targetVelocity` until the total target distance is covered.
 */
#pragma once
#ifndef ACCELERATEDISTANCE_STEP_H
#define ACCELERATEDISTANCE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Drive straight with a linear velocity ramp-up phase.
 */
class AccelerateStraightLine : public AutoStep
{
public:
    /**
     * @brief Full constructor.
     * @param drive               Drive subsystem reference.
     * @param targetDistance      Total distance to travel (m).
     * @param targetVelocity      Maximum velocity after acceleration (m/s).
     * @param acceleration        Acceleration rate (m/s²).
     * @param accelerationDistance Distance over which to ramp up (m).
     */
    AccelerateStraightLine(Drive &drive,
                           float targetDistance,
                           float targetVelocity,
                           float acceleration,
                           float accelerationDistance)
        : _drive(drive),
          _target(targetDistance),
          _finalVelocity(targetVelocity),
          _acceleration(acceleration),
          _accelerationDistance(accelerationDistance){}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
    AccelerateStraightLine(Drive &drive)
        : _drive(drive) {}

    void start() override
    {
        lastTime = millis();
        _velocity = 0;
        _startDistance = _drive.getDistance(); // or encoder average
        _drive.setSpeed(_velocity);
    }

    void update() override
    {
        float dt = 1000.0/(millis() - lastTime);
        //If the robot is going below target velocity and less distance than target acceleration period is done
        if( _velocity < _finalVelocity && _drive.getDistance() - _startDistance >= _accelerationDistance ){
            _velocity += _acceleration * dt;
            _drive.setSpeed(_velocity);
        }

    }

    bool isFinished() const override
    {
        // Serial.print("Drive Distance: Distance Elapsed ");
        // Serial.println(_drive.getDistance());
        return (_drive.getDistance() - _startDistance) >= _target;
    }

    void end()
    {
        _drive.hardSetSpeed(0);
    }

    /**
     * @brief Re-configure the step for reuse.
     * @param targetDistance       Total distance to travel (m).
     * @param targetVelocity       Cruising velocity (m/s).
     * @param acceleration         Acceleration rate (m/s²).
     * @param accelerationDistance Ramp-up distance (m).
     */
    void configure(float targetDistance, float targetVelocity, float acceleration, float accelerationDistance)
    {
        _target = targetDistance;
        _finalVelocity = targetVelocity;
        _acceleration = acceleration;
        _accelerationDistance = accelerationDistance;
    }

private:
    Drive &_drive;                       ///< Drive subsystem reference
    float _target = 0;                   ///< Total distance to cover (m)
    float _velocity = 0;                 ///< Current commanded velocity (m/s)
    float _finalVelocity = 0;            ///< Target velocity at end of ramp (m/s)
    float _startDistance = 0;            ///< Odometry distance at step start (m)
    float _acceleration = 0;             ///< Acceleration rate (m/s²)
    float _accelerationDistance = 0;     ///< Ramp-up distance (m)
    unsigned long lastTime;              ///< millis() snapshot for dt calculation
};

#endif