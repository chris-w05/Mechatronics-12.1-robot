/**
 * @file ParallelStep.hpp
 * @brief `AutoStep` that runs multiple child steps concurrently.
 *
 * Two completion policies are supported via `ParallelFinishMode`:
 *   - `ALL` — waits for every child to finish.
 *   - `ANY` — finishes as soon as the first child completes.
 *
 * `ParallelStep` takes ownership of all child pointers and deletes them in the destructor.
 */
// ParallelStep.hpp
#ifndef PARALLEL_STEP_H
#define PARALLEL_STEP_H

#include "Autonomous/AutoStep.h"
#include <vector>

/** @brief Determines when a `ParallelStep` considers itself complete. */
enum class ParallelFinishMode
{
    ALL, ///< Complete when ALL child steps have finished
    ANY  ///< Complete when ANY child step finishes first
};

/**
 * @brief Runs a collection of `AutoStep` children in parallel each tick.
 */
class ParallelStep : public AutoStep
{
public:
    /**
     * @brief Construct from an array of step pointers (ownership transferred).
     * @param steps  Array of `AutoStep*` to run in parallel.
     * @param count  Number of steps in the array.
     * @param mode   Completion policy (default: ALL).
     */
    ParallelStep(AutoStep **steps, int count, ParallelFinishMode mode = ParallelFinishMode::ALL)
        : _mode(mode)
    {
        for (int i = 0; i < count; ++i)
        {
            if (steps[i])
                _steps.push_back(steps[i]);
        }
    }

    /**
     * @brief Construct from a vector of step pointers (ownership transferred).
     * @param steps  Vector of `AutoStep*` to run in parallel.
     * @param mode   Completion policy (default: ALL).
     */
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

    /** @brief End all child steps and clean them up. */
    void end() override
    {
        for (AutoStep *s : _steps)
        {
            s->end();
        }
    }


private:
    std::vector<AutoStep *> _steps; ///< Owned collection of child steps
    ParallelFinishMode _mode;       ///< Completion policy
    bool _started = false;          ///< True after start() has been called
};

#endif // PARALLEL_STEP_H
