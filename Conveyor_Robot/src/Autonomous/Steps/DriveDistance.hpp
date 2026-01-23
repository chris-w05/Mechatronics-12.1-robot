#ifndef DRIVEDISTANCE_STEP_H
#define DRIVEDISTANCE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "Devices/Encoder.hpp"

class DriveDistance : public AutoStep{
public:
    DriveDistance(Drive& drive,
                float targetDistance,
                float targetVelocity)
        : _drive(drive),
          _target(targetDistance),
          _velocity(targetVelocity) {}

    DriveDistance(Drive &drive)
        : _drive(drive){}

    void start() override
    {
        _startDistance = _drive.getDistance(); // or encoder average
    }

    void update() override
    {
        _drive.setSpeed(_velocity);
    }

    bool isFinished() const override
    {
        return (_drive.getDistance() - _startDistance) >= _target;
    }

    void end(){
        _drive.setSpeed(0);
    }

    void configure(float targetDistance, float targetVelocity){
        _target = targetDistance;
        _velocity = targetVelocity;

    }

private:
    Drive &_drive;
    float _target = 0;
    float _velocity = 0;
    float _startDistance = 0;
};


#endif