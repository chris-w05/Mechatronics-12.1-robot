#pragma once
#ifndef FIRE_STEP_H
#define FIRE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Shooter.hpp"

class FireStep : public AutoStep
{
public:
    FireStep( Shooter& shooter, unsigned long time, bool reversed)
        : 
        _shooter(shooter),
        _time(time),
        _reversed(reversed) {}


    void start() override
    {
        _startTime = millis();
        _shooter.fire();
    }

    void update() override
    {
    }

    bool isFinished() const override
    {
        Serial.print("Time until finished: ");
        Serial.println(_time + _startTime - millis());
         return (millis() - _startTime) > _time;
    }

    void end() override
    {
        _shooter.stop();
    }

    void configure( long time, bool reversed)
    {
        _time = time;
        _reversed = reversed;
    }

private:
    Shooter& _shooter;
    unsigned long _time = 0;
    bool _reversed = false;
    long _startTime = 0;
};

#endif