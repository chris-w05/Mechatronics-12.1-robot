/**
 * @file ReplanStep.hpp
 * @brief An `AutoStep` that repeatedly queries a planner for the next child step.
 *
 * Implements a "planner loop": call a planning function to get the next step,
 * run it to completion, then call the planner again.  When the planner returns
 * `nullptr` the loop is finished.
 *
 * **Ownership model (non-owning):** `ReplanStep` does *not* delete returned
 * child steps.  Child steps must outlive this object — store them as member
 * variables of the planner or as statically allocated objects so they can be
 * re-used across multiple calls without heap allocation.
 */
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

/**
 * @brief Drives a looping planning function that supplies child steps on demand.
 *
 * The planner function (`PlanFn`) is called after each child step finishes;
 * returning `nullptr` signals that the sequence is complete.
 */
class ReplanStep : public AutoStep
{
public:
    /** @brief Signature of the planning function: `AutoStep* plan(void* context)`. */
    using PlanFn = AutoStep *(*)(void *ctx);

    /**
     * @brief Construct the ReplanStep.
     * @param ctx  Opaque context pointer forwarded to the planner.
     * @param fn   Planning function called at each transition.
     */
    ReplanStep(void *ctx, PlanFn fn) : _ctx(ctx), _plan(fn) {}

    /** @brief Begin the loop, requesting the first child from the planner. */
    void start() override
    {
        _done = false;
        _child = nullptr;
        startNextChild();
    }

    /** @brief Tick the active child; when it finishes, request the next one. */
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

    /** @brief @return `true` after the planner returns `nullptr` or `end()` is called. */
    bool isFinished() const override { return _done; }

    /** @brief End the currently active child (if any) and mark this step done. */
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
    /** @brief Ask the planner for the next step and call `start()` on it. */
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

    void *_ctx = nullptr;        ///< Opaque context forwarded to planner
    PlanFn _plan = nullptr;      ///< Planning function called at each step transition

    AutoStep *_child = nullptr;  ///< Currently active child step (NON-OWNING)
    bool _done = false;          ///< True when the planning loop has completed
};

#endif
