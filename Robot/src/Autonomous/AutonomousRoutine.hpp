#ifndef AUTONOMOUS_ROUTINE_H
#define AUTONOMOUS_ROUTINE_H

#include "AutoStep.h"
#include "Config.hpp"

class AutonomousRoutine
{
private:
    AutoStep *_steps[MAX_STEPS];
    int _count = 0;
    int _index = 0;

public:
    AutonomousRoutine()
    {
        for (int i = 0; i < MAX_STEPS; ++i)
        {
            _steps[i] = nullptr;
        }
    }

    ~AutonomousRoutine()
    {
        for (int i = 0; i < _count; ++i)
        {
            delete _steps[i];
        }
    }

    // Start the routine from the beginning.
    // If a step is currently running, end it first to leave it in a clean state.
    void start()
    {
        // If currently running a step, end it so we restart cleanly.
        if (_index >= 0 && _index < _count && _steps[_index])
        {
            _steps[_index]->end();
        }

        _index = 0;
        if (_count > 0 && _steps[0])
        {
            _steps[0]->start();
        }
    }

    // Call repeatedly from main loop to drive the routine
    void update()
    {
        if (_index >= _count)
        {
            return;
        }

        AutoStep *step = _steps[_index];
        if (!step)
            return;

        step->update();

        if (step->isFinished())
        {
            Serial.println("Step is finished");
            step->end();
            _index++;
            if (_index < _count && _steps[_index])
            {
                _steps[_index]->start();
            }
        }
    }

    // Reset the routine so it can run again:
    // - end the currently active step (if any)
    // - set index to 0 and start the first step (if any)
    void reset()
    {
        // End the currently running step to put it into a clean state
        if (_index >= 0 && _index < _count && _steps[_index])
        {
            _steps[_index]->end();
        }

        _index = 0;

        // Start the first step so the next update() continues execution
        if (_count > 0 && _steps[0])
        {
            _steps[0]->start();
        }
    }

    void stop()
    {
        if (_index < _count && _steps[_index])
        {
            _steps[_index]->end();
        }
        _index = _count; // mark complete
    }

    // Add a step to the routine (ownership transferred to this object)
    void add(AutoStep *step)
    {
        if (_count >= MAX_STEPS)
        {
            Serial.print("AutonomousRoutine - add - Max steps reached!");
            return;
        }
        _steps[_count++] = step;
    }
};

#endif
