/**
 * @file AutonomousRoutine.hpp
 * @brief Sequential container that drives an ordered list of `AutoStep` objects.
 *
 * Steps are added with `add()` (ownership is transferred) and are executed
 * one after the other: when the active step reports `isFinished()`, the
 * routine calls `end()` on it, increments its internal index, and starts the
 * next step via `start()`.
 *
 * The maximum number of steps is bounded by `MAX_STEPS` from Config.hpp.
 */
#ifndef AUTONOMOUS_ROUTINE_H
#define AUTONOMOUS_ROUTINE_H

#include "AutoStep.h"
#include "Config.hpp"

/**
 * @brief Drives an ordered sequence of `AutoStep` objects to completion.
 *
 * Owns all steps added via `add()` and deletes them in the destructor.
 */
class AutonomousRoutine
{
private:
    AutoStep *_steps[MAX_STEPS]; ///< Fixed-size array of step pointers
    int _count = 0;              ///< Number of steps that have been added
    int _index = 0;              ///< Index of the currently active step

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

    /**
     * @brief Start the routine from the first step.
     *
     * If a step is currently in progress its `end()` is called first so the
     * subsystems it owns are left in a clean state.
     */
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

    /** @brief Advance the routine by one tick — call every loop iteration. */
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
            // Serial.println("Step is finished");
            step->end();
            _index++;
            if (_index < _count && _steps[_index])
            {
                _steps[_index]->start();
            }
        }
    }

    /** @brief @return Total number of steps currently in the routine. */
    int getCount(){
        return _count;
    }

    /**
     * @brief End the active step (if any) and rewind the index to 0.
     *
     * Does **not** restart the first step — call `start()` afterwards if needed.
     */
    void reset()
    {
        // End the currently running step to put it into a clean state
        if (_index >= 0 && _index < _count && _steps[_index])
        {
            _steps[_index]->end();
        }

        _index = 0;
    }

    /**
     * @brief End the active step and mark the routine as complete.
     */
    void stop(){
        if (_index < _count && _steps[_index])
        {
            _steps[_index]->end();
        }
        _index = _count; // mark complete
    }

    /**
     * @brief Append a step to the end of the routine (transfers ownership).
     * @param step Heap-allocated step — deleted by the routine's destructor.
     */
    void add(AutoStep *step)
    {
        if (_count >= MAX_STEPS)
        {
            // Serial.print("AutonomousRoutine - add - Max steps reached!");
            return;
        }
        _steps[_count++] = step;
    }

    /**
     * @brief Delete the step at `index` and shift remaining steps left.
     *
     * If the removed step is currently running, its `end()` is called first.
     * @param index  Zero-based position of the step to remove.
     */
    void removeAt(int index)
    {
        if (index < 0 || index >= _count)
        {
            return;
        }

        // If we're removing the currently running step, end it
        if (index == _index && _steps[index])
        {
            _steps[index]->end();
        }

        delete _steps[index];

        // Shift remaining steps left
        for (int i = index; i < _count - 1; ++i)
        {
            _steps[i] = _steps[i + 1];
        }

        _steps[_count - 1] = nullptr;
        _count--;

        // Fix index if needed
        if (_index > index)
        {
            _index--;
        }
        else if (_index >= _count)
        {
            _index = _count;
        }
    }

    /**
     * @brief Delete all steps and reset the routine to an empty state.
     */
    void clear()
    {
        // End current step if running
        if (_index >= 0 && _index < _count && _steps[_index])
        {
            _steps[_index]->end();
        }

        for (int i = 0; i < _count; ++i)
        {
            delete _steps[i];
            _steps[i] = nullptr;
        }

        _count = 0;
        _index = 0;
    }
};

#endif
