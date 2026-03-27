/**
 * @file DeferredStep.hpp
 * @brief An `AutoStep` wrapper whose child step is chosen at runtime via a factory.
 *
 * Useful when the correct action can only be decided at the moment the step
 * begins — for example, switching from a normal mining sequence to a
 * silverfish-kill sequence based on the color sensor reading seen just before
 * `start()` is called.
 *
 * The factory function receives a user-supplied context pointer and returns a
 * heap-allocated `AutoStep`.  `DeferredStep` owns the returned child and
 * deletes it on destruction.
 */
//For steps like mining a block. The step has to defer to different cases depending on the situation. 
// EX: If the robot detects a silverfish block, it changes the routine to add a kill step


#ifndef DEFERRED_STEP_H
#define DEFERRED_STEP_H

#include "AutoStep.h"

/**
 * @brief Wraps a lazily-constructed child step that is created by a factory at `start()` time.
 */
class DeferredStep : public AutoStep
{
public:
    /** @brief Signature of the factory function: `AutoStep* factory(void* context)`. */
    using FactoryFn = AutoStep *(*)(void *ctx);

    /**
     * @brief Construct the deferred step.
     * @param ctx  Opaque context pointer forwarded to the factory.
     * @param fn   Factory function that creates the child step.
     */
    DeferredStep(void *ctx, FactoryFn fn) : _ctx(ctx), _fn(fn) {}

    /** @brief Invoke the factory and start the child step. */
    void start() override
    {
        _child = (_fn) ? _fn(_ctx) : nullptr;
        if (_child)
            _child->start();
    }

    /** @brief Forward one tick to the child step (no-op if already finished). */
    void update() override
    {
        if (_child && !_child->isFinished())
            _child->update();
    }

    /** @brief @return `true` if the child is done, or if no child was created. */
    bool isFinished()const override
    {
        return (_child == nullptr) ? true : _child->isFinished();
    }

    /** @brief Forward `end()` to the child step. */
    void end() override
    {
        if (_child)
            _child->end();
    }

    /** @brief Destroy the owned child step, if one was created. */
    ~DeferredStep() override { delete _child; }

private:
    void *_ctx = nullptr;       ///< Opaque context pointer passed to the factory
    FactoryFn _fn = nullptr;    ///< Factory function that creates the child step
    AutoStep *_child = nullptr; ///< Heap-allocated child step (owned by this object)
};

#endif