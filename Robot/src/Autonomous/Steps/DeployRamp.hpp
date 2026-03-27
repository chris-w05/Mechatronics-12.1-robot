/**
 * @file DeployRamp.hpp
 * @brief Instantaneous `AutoStep` that deploys the miner's block-collection ramp.
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"

/**
 * @brief Calls `Miner::deployRamp()` in `start()` and finishes immediately.
 */
class DeployRamp : public AutoStep
{
public:
    /**
     * @brief Construct the step.
     * @param miner  Miner subsystem that owns the ramp servo.
     */
    DeployRamp(Miner& miner ):
    miner(miner){
        
    }

    void start() override
    {
        miner.deployRamp();
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

    void configure(){
    }

private:
    Miner &miner; ///< Reference to the Miner subsystem
};