#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"

class MineBlockStep : public AutoStep
{
public:
    MineBlockStep(Miner& miner, int hits ):
    miner(miner), _numberHits(hits) {
        
    }

    void start() override
    {
        miner.mine(_numberHits);
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
    }

    void configure(int hits){
        _numberHits = hits;
    }

private:
    Miner &miner;

    short _numberHits = 5;
    bool hitsMet = false;
};