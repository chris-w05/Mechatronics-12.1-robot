#ifndef MOVEARMTOPOS_STEP_H
#define MOVEARMTOPOS_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Arm.hpp"
#include "Devices/Encoder.hpp"

class MoveArmToPos : public AutoStep
{
public:
    MoveArmToPos(Drive drive,
                  float targetMeters,
                  float targetVelocity)
        : _drive(drive),
          _target(targetMeters),
          _velocity(targetVelocity) {}

    void start() override
    {
        _initialPosition = _arm.getPosition();
    }

    void update() override
    {
        _drive.setSpeed(_velocity);
    }

    bool isFinished() const override
    {
        if (_target >= _initialPosition) return _arm.getPosition() >= _target;
        else return _arm.getPosition() <= _target;
    }

private:
    Drive &_drive;
    Arm &_arm;
    float _target;
    float _velocity;
    int _initialPosition;
};

#endif
