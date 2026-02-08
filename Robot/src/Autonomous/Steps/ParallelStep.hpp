// ParallelStep.hpp
#ifndef PARALLEL_STEP_H
#define PARALLEL_STEP_H

#include "Autonomous/AutoStep.h"
#include <vector>

enum class ParallelFinishMode
{
    ALL, // finish when all sub-steps finished
    ANY  // finish when any sub-step finishes
};

class ParallelStep : public AutoStep
{
public:
    // take ownership of pointers (will delete in destructor)
    ParallelStep(AutoStep **steps, int count, ParallelFinishMode mode = ParallelFinishMode::ALL)
        : _mode(mode)
    {
        for (int i = 0; i < count; ++i)
        {
            if (steps[i])
                _steps.push_back(steps[i]);
        }
    }

    // convenience ctor for vector
    ParallelStep(const std::vector<AutoStep *> &steps, ParallelFinishMode mode = ParallelFinishMode::ALL)
        : _mode(mode), _steps(steps) {}

    ~ParallelStep() override
    {
        for (AutoStep *s : _steps)
            delete s;
        _steps.clear();
    }

    void start() override
    {
        _started = true;
        for (AutoStep *s : _steps)
        {
            if (s)
                s->start();
        }
    }

    void update() override
    {
        if (!_started)
            return;

        for (AutoStep *s : _steps)
        {
            if (s && !s->isFinished())
            {
                s->update();
            }
        }
    }

    bool isFinished() const override
    {
        if (!_started)
            return false;

        if (_mode == ParallelFinishMode::ALL)
        {
            for (AutoStep *s : _steps)
            {
                if (s && !s->isFinished())
                    return false;
            }
            return true;
        }
        else
        { // ANY
            for (AutoStep *s : _steps)
            {
                if (s && s->isFinished())
                    return true;
            }
            return false;
        }
    }

    // optional: call to cancel (call stop on child steps if you implement stop())
    void end() override
    {
        for (AutoStep *s : _steps)
        {
            s->end();
        }
    }


private:
    std::vector<AutoStep *> _steps;
    ParallelFinishMode _mode;
    bool _started = false;
};

#endif // PARALLEL_STEP_H
