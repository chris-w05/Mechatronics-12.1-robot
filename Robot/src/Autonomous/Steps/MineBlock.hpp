/**
 * @file MineBlock.hpp
 * @brief `AutoStep` that commands the Miner to complete a fixed number of press cycles.
 *
 * Calls `Miner::startMining(hits)` in `start()` and waits until
 * `Miner::isDoneMining()` returns `true`.
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"

/**
 * @brief Mine a block by executing a set number of servo press cycles.
 */
class MineBlock : public AutoStep
{
public:
    /**
     * @brief Construct the step.
     * @param drive  Drive subsystem reference
     * @param miner  Miner subsystem reference.
     * @param hits   Number of press cycles to complete.
     */
    MineBlock(Drive &drive, Miner &miner, int hits) : drive(drive), miner(miner), _numberHits(hits) 
    {
    }

    void start() override
    {
        miner.startMining(_numberHits);
    }

    void update() override
    { 

    }

    bool isFinished() const override
    {
        return miner.isDoneMining();
    }

    void end() override
    {
        //Set the measured pose to the desired pose. 
        drive.setDesiredPositionsToCurrent();
        miner.store();
    }

    /**
     * @brief Re-configure the hit count for reuse.
     * @param hits  New target hit count.
     */
    void configure(int hits){
        _numberHits = hits;
    }

private:
    Drive &drive;
    Miner &miner;              ///< Miner subsystem reference
    short _numberHits = 5;     ///< Target number of press cycles
    bool hitsMet = false;      ///< (unused — completion tracked by Miner)
};