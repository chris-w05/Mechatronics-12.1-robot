#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

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