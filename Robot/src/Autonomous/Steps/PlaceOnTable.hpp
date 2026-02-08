#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "utils/Strategy.hpp"

class PlaceOnTableStep : public AutoStep
{
    public:
        PlaceOnTableStep(){

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

        void configure(Strategy::CycleType whereToPlace){
            //Change variables to make sure arm and other subsystems are able to manipulate the block properly
            _whereToPlace = whereToPlace;
        }

    private:
        Strategy::CycleType _whereToPlace = Strategy::CycleType::PICKAXE_HANDLE;
};