/**
 * @file AdvanceCycleStep.hpp
 * @brief Instantaneous `AutoStep` that advances the `Strategy` craft cycle by one.
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Calls `Strategy::advanceCycle()` once in `start()` and immediately finishes.
 */
class AdvanceCycleStep : public AutoStep
{
public:
    void start() override
    {
        // Serial.println("AdvanceCycleStep: advancing strategy cycle");
        Strategy::getInstance().advanceCycle();
        _done = true;
    }

    void update() override {}
    bool isFinished() const override { return _done; }
    void end() override { }

private:
    bool _done = false; ///< True after start() calls advanceCycle()
};