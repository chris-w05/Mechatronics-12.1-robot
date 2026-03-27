/**
 * @file IndexBlock.hpp
 * @brief `AutoStep` that routes a newly acquired block to the correct hopper slot.
 *
 * The direction (LEFT / RIGHT / REJECT) is set via `configure()` and reflects
 * the decision made by `Strategy::whereToIndexBlock()`.
 */
#ifndef PLACE_BLOCK_STEP_H
#define PLACE_BLOCK_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "utils/Strategy.hpp"

/**
 * @brief Push the block into the left or right slot of the hopper based on `Strategy::IndexDirection`.
 */
class IndexBlockStep : public AutoStep
{
public:
    /**
     * @brief Construct with a known index direction.
     * @param dir  Slot direction: LEFT, RIGHT, or REJECT.
     */
    IndexBlockStep(Strategy::IndexDirection dir
                  )
        : _dir(dir) {}

    IndexBlockStep(){
        
    }

    void start() override
    {
    }

    void update() override
    {
        //Logic to make servo push block to either side of the hopper, then reset. Governed by _dir for direction to index
    }

    bool isFinished() const override
    {
        return true;
    }

    void end() override
    {
    }

    /**
     * @brief Re-configure the index direction for reuse.
     * @param dir  New slot direction.
     */
    void configure(Strategy::IndexDirection dir){
        _dir = dir;
    }

private:
    Strategy::IndexDirection _dir; ///< Index direction resolved by Strategy
};

#endif