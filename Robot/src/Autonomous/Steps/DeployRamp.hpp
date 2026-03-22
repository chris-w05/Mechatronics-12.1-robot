#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"

class DeployRamp : public AutoStep
{
public:
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
    Miner &miner;
};