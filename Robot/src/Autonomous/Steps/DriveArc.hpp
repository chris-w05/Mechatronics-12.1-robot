#ifndef DRIVEARC_STEP_H
#define DRIVEARC_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class DriveArc : public AutoStep{
public:

    /**
     * @param direction true -> Counter-clockwise | false -> clockwise
     */
    DriveArc(Drive& drive,
                float targetAngle,
                float targetVelocity,
                float radius, 
                bool direction)
        : _drive(drive),
          _target(targetAngle),
          _velocity(targetVelocity) {}

    DriveArc(Drive &drive)
        : _drive(drive){}

    DriveArc();

    void start() override
    {
        _startAngle = _drive.getPose().heading; // or encoder average
    }

    void update() override
    {
        if (_direction){
            _drive.followRadiusCCW( _velocity, _radius);
        }
        else{
            _drive.followRadiusCCW(_velocity, _radius);
        }
    }

    bool isFinished() const override
    {
        return (_drive.getPose().heading - _startAngle) >= _target;
    }

    void end(){
        _drive.setSpeed(0);
    }

    void configure(float targetDistance, float targetVelocity, float radius, bool direction){
        _target = targetDistance;
        _velocity = targetVelocity;
        _radius = radius;
        _direction = direction;

    }

private:
    Drive &_drive;
    float _target = 0;
    float _velocity = 0;
    float _radius = 10;
    bool _direction = 0;
    float _startAngle = 0;
};


#endif