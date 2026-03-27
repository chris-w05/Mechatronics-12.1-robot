/**
 * @file DelayStep.hpp
 * @brief `AutoStep` that blocks for a fixed number of milliseconds.
 */
#pragma once
#ifndef DELAY_STEP_H
#define DELAY_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Shooter.hpp"

/**
 * @brief Waits for a configurable duration (ms) then completes.
 */
class DelayStep : public AutoStep
{
public:
    /**
     * @brief Construct with a fixed delay.
     * @param time  Duration to wait (ms).
     */
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
        // Serial.print("Time until finished: ");
        // Serial.println(_time + _startTime - millis());
        return (millis() - _startTime) > _time;
    }

    void end() override
    {

    }

    /**
     * @brief Re-configure the delay for reuse.
     * @param time  New delay duration (ms).
     */
    void configure(long time)
    {
        _time = time;
    }

private:
    unsigned long _time = 0;      ///< Delay duration (ms)
    unsigned long _startTime = 0; ///< millis() at step start
};

#endif