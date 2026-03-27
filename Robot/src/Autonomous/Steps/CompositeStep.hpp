/**
 * @file CompositeStep.hpp
 * @brief `AutoStep` that runs a heap-owned sequence of child steps in order.
 *
 * Children are passed to the constructor (or `add()`) as raw pointers;
 * `CompositeStep` takes ownership and `delete`s each one when it finishes.
 * When the last child completes, `isFinished()` returns `true`.
 */
#ifndef COMPOSITE_STEP_H
#define COMPOSITE_STEP_H

#include "Autonomous/AutoStep.h"
#include "Config.hpp"

const int MAX_SUB_STEPS = 10; // Adjust based on your max expected sub-steps per composite

/**
 * @brief Sequences heap-allocated child `AutoStep` objects, deleting each upon completion.
 */
class CompositeStep : public AutoStep
{
public:
    template <typename... Ts>
    CompositeStep *Sequence(Ts *...steps)
    {
        AutoStep *arr[] = {steps...};
        return new CompositeStep(arr, sizeof...(steps));
    }

    /**
     * @brief Construct from an array of step pointers (ownership transferred).
     * @param steps  Array of `AutoStep*` to run in order.
     * @param count  Number of steps in the array.
     */
    CompositeStep(AutoStep *steps[], uint8_t count)
        : _children(nullptr), _count(count), _currentIndex(-1), _started(false)
    {
        if (count == 0)
        {
            _children = nullptr;
            return;
        }

        // Allocate array to persist pointers beyond caller's stack.
        _children = new AutoStep *[count];
        for (uint8_t i = 0; i < count; ++i)
        {
            _children[i] = steps[i];
        }
    }

    ~CompositeStep()
    {
        // If there are remaining children, call end() and delete them.
        if (_children)
        {
            for (uint8_t i = 0; i < _count; ++i)
            {
                if (_children[i])
                {
                    // If the step hasn't been ended already, call end() so resources are released.
                    _children[i]->end();
                    delete _children[i];
                    _children[i] = nullptr;
                }
            }
            delete[] _children;
            _children = nullptr;
        }
    }

    void start() override
    {
        if (_started)
            return;
        _started = true;

        if (_count == 0)
        {
            _currentIndex = -1; // nothing to do
            return;
        }

        _currentIndex = 0;
        if (_children[0])
        {
            _children[0]->start();
        }
    }

    void update() override
    {
        if (!_started)
            start();
        if (_count == 0)
            return;
        if (_currentIndex < 0 || _currentIndex >= _count)
            return;

        AutoStep *cur = _children[_currentIndex];
        if (!cur)
        {
            // Shouldn't happen, but advance to next
            _currentIndex++;
            if (_currentIndex < _count && _children[_currentIndex])
            {
                _children[_currentIndex]->start();
            }
            return;
        }

        // Let the current step run its logic
        cur->update();

        // If it finished, call end(), delete it, and advance to the next step
        if (cur->isFinished())
        {
            cur->end(); 
            delete cur;
            _children[_currentIndex] = nullptr;

            _currentIndex++;
            if (_currentIndex < _count)
            {
                if (_children[_currentIndex])
                {
                    _children[_currentIndex]->start();
                }
            }
        }
    }

    bool isFinished() const override
    {
        if (_count == 0)
            return true;
        if (_currentIndex == -1)
            return false; // not started yet
        return (_currentIndex >= _count);
    }

    void end() override{
        // End & delete current and remaining steps
        if (_children)
        {
            for (uint8_t i = 0; i < _count; ++i)
            {
                if (_children[i])
                {
                    _children[i]->end();
                    delete _children[i];
                    _children[i] = nullptr;
                }
            }
            delete[] _children;
            _children = nullptr;
        }
        _count = 0;
        _currentIndex = -1;
        _started = true;
    }

    /**
     * @brief Append a step to the end of the sequence (ownership transferred).
     * @param step  Heap-allocated step to append.
     */
    void add(AutoStep *step)
    {
        if (_count < MAX_STEPS)
        {
            _children[_count++] = step;
        }
    }

private:
    AutoStep **_children;   ///< Owned heap-allocated array of child step pointers
    uint8_t _count;         ///< Total number of child steps
    int16_t _currentIndex;  ///< -1 = not started, >= 0 = index of active child
    bool _started;          ///< True after start() has been called
};

#endif // COMPOSITE_STEP_H