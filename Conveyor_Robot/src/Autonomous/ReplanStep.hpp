#ifndef REPLAN_STEP_H
#define REPLAN_STEP_H

// ReplanStep.hpp
// Runs a "planner loop":
//   - ask a planner for the next AutoStep to run
//   - run it until finished
//   - ask again
//
// IMPORTANT (Arduino Mega reliability):
// This version is NON-OWNING. It does NOT delete the child step.
// That allows you to avoid heap allocation/free during the match.
//
// Contract:
// - Your planner must return pointers to steps that live for the duration of the match
//   (e.g., member variables inside Planner, or statically allocated objects).
// - Those steps must be reusable: calling start() should reset them.

#include "AutoStep.h"

class ReplanStep : public AutoStep
{
public:
    using PlanFn = AutoStep *(*)(void *ctx);

    ReplanStep(void *ctx, PlanFn fn) : _ctx(ctx), _plan(fn) {}

    void start() override
    {
        _done = false;
        _child = nullptr;
        startNextChild();
    }

    void update() override
    {
        if (_done)
            return;

        if (!_child)
        {
            _done = true;
            return;
        }

        _child->update();

        if (_child->isFinished())
        {
            _child->end();
            _child = nullptr; // NON-OWNING: do NOT delete
            startNextChild();
        }
    }

    bool isFinished() const override { return _done; }

    void end() override
    {
        if (_child)
        {
            _child->end();
            _child = nullptr;
        }
        _done = true;
    }

private:
    void startNextChild()
    {
        if (!_plan)
        {
            _done = true;
            return;
        }

        _child = _plan(_ctx);

        // Convention: nullptr means "finished planning".
        if (!_child)
        {
            _done = true;
            return;
        }

        _child->start();
    }

    void *_ctx = nullptr;
    PlanFn _plan = nullptr;

    AutoStep *_child = nullptr; // NON-OWNING
    bool _done = false;
};

#endif
