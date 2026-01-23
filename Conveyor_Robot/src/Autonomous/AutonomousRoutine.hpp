//Sequentially runs a list of AutoSteps. This defines the procedure of an autonomous by ordering steps and choosing which step to run

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


    void start()
    {
        _index = 0;
        if (_count > 0)
        {
            _steps[0]->start();
        }
    }

    void update()
    {
        if (_index >= _count)
        {
            return;
        }

        AutoStep *step = _steps[_index];
        step->update();

        if (step->isFinished())
        {
            step->end();
            _index++;
            if (_index < _count)
            {
                _steps[_index]->start();
            }
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