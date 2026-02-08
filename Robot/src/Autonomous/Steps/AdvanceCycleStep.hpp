#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class AdvanceCycleStep : public AutoStep
{
public:
    void start() override
    {
        Serial.println("AdvanceCycleStep: advancing strategy cycle");
        Strategy::getInstance().advanceCycle();
        _done = true;
    }

    void update() override {}
    bool isFinished() const override { return _done; }
    void end() override { }

private:
    bool _done = false;
};