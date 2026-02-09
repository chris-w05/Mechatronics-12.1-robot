#pragma once
#ifndef DELAY_STEP_H
#define DELAY_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Shooter.hpp"

class DelayStep : public AutoStep
{
public:
    DelayStep( unsigned long time)
        : 
          _time(time){}

    void start() override
    {
        _startTime = millis();
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

    }

    void configure(long time)
    {
        _time = time;
    }

private:
    unsigned long _time = 0;
    unsigned long _startTime = 0;
};

#endif