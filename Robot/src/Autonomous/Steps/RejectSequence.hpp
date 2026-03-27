/**
 * @file RejectSequence.hpp
 * @brief `AutoStep` stub that executes the block-rejection sequence.
 *
 * Called when `Strategy::whereToIndexBlock()` returns `REJECT`.
 * Currently completes immediately; rejection servo logic to be implemented.
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Placeholder step that discards an unwanted block (completes immediately).
 */
class RejectBlockStep : public AutoStep
{
    public:
        RejectBlockStep(){
            
        }

        void start() override
        {
        }

        void update() override
        {
        }

        bool isFinished() const override
        {
            return true;
        }

        void end() override
        {
        }

    private:
};