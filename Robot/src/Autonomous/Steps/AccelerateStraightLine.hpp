#pragma once
#ifndef ACCELERATEDISTANCE_STEP_H
#define ACCELERATEDISTANCE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class AccelerateStraightLine : public AutoStep
{
public:
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
            _drive.setSpeed(_velocity, false);
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

    void configure(float targetDistance, float targetVelocity, float acceleration, float accelerationDistance)
    {
        _target = targetDistance;
        _finalVelocity = targetVelocity;
        _acceleration = acceleration;
        _accelerationDistance = accelerationDistance;
    }

private:
    Drive &_drive;
    float _target = 0;
    float _velocity = 0;
    float _finalVelocity= 0;
    float _startDistance = 0;
    float _acceleration = 0;
    float _accelerationDistance = 0;

    unsigned long lastTime;
};

#endif