/**
 * @file FireStep.hpp
 * @brief `AutoStep` that fires the shooter for a fixed duration, then stops it.
 */
#pragma once
#ifndef FIRE_STEP_H
#define FIRE_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Shooter.hpp"

/**
 * @brief Triggers `Shooter::fire()` and holds for `_time` ms before calling `Shooter::stop()`.
 */
class FireStep : public AutoStep
{
public:
    /**
     * @brief Construct the fire step.
     * @param shooter   Shooter subsystem reference.
     * @param time      Duration to hold the fire command (ms).
     * @param reversed  Reserved for future reverse-fire support (unused).
     */
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

        // Serial.print("Time until finished: ");
        // Serial.println(_time + _startTime - millis());
        return (millis() - _startTime) > _time;
    }

    void end() override
    {
        _shooter.stop();
    }

    /**
     * @brief Re-configure the step for reuse.
     * @param time      New fire duration (ms).
     * @param reversed  Reverse-fire flag (unused).
     */
    void configure( long time, bool reversed)
    {
        _time = time;
        _reversed = reversed;
    }

private:
    Shooter& _shooter;         ///< Shooter subsystem reference
    unsigned long _time = 0;   ///< Fire duration (ms)
    bool _reversed = false;    ///< Reverse-fire flag (reserved, unused)
    long _startTime = 0;       ///< millis() at step start
};

#endif