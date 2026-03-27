/**
 * @file PlaceOnTable.hpp
 * @brief `AutoStep` stub that places a block on the crafting table.
 *
 * The target slot is set via `configure()` using a `Strategy::CycleType`.
 * The step completes immediately (arm/servo logic to be implemented).
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "utils/Strategy.hpp"

/**
 * @brief Placeholder step for placing a block on the crafting table at the correct cycle position.
 */
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

        /**
         * @brief Set the crafting table slot to target.
         * @param whereToPlace  Slot type from `Strategy::CycleType`.
         */
        void configure(Strategy::CycleType whereToPlace){
            //Change variables to make sure arm and other subsystems are able to manipulate the block properly
            _whereToPlace = whereToPlace;
        }

    private:
        Strategy::CycleType _whereToPlace = Strategy::CycleType::PICKAXE_HANDLE; ///< Target crafting table slot
};