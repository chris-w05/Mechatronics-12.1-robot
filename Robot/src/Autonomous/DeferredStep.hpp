//For steps like mining a block. The step has to defer to different cases depending on the situation. 
// EX: If the robot detects a silverfish block, it changes the routine to add a kill step


#ifndef DEFERRED_STEP_H
#define DEFERRED_STEP_H

#include "AutoStep.h"

class DeferredStep : public AutoStep
{
public:
    using FactoryFn = AutoStep *(*)(void *ctx);

    DeferredStep(void *ctx, FactoryFn fn) : _ctx(ctx), _fn(fn) {}

    void start() override
    {
        _child = (_fn) ? _fn(_ctx) : nullptr;
        if (_child)
            _child->start();
    }

    void update() override
    {
        if (_child && !_child->isFinished())
            _child->update();
    }

    bool isFinished()const override
    {
        return (_child == nullptr) ? true : _child->isFinished();
    }

    void end() override
    {
        if (_child)
            _child->end();
    }

    ~DeferredStep() override { delete _child; }

private:
    void *_ctx = nullptr;
    FactoryFn _fn = nullptr;
    AutoStep *_child = nullptr;
};

#endif