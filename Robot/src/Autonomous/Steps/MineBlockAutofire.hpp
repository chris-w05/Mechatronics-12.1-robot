#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/Shooter.hpp"

class MineBlockAutofire : public AutoStep
{
public:
    MineBlockAutofire(Miner& miner, Shooter& shooter, float timeSeconds):
    miner(miner), shooter(shooter), desiredTime(timeSeconds*1000.0) {
        
    }

    void start() override
    {
        startTime = millis();
        miner.startMiningIndefinitely();
        shooter.autoFire();
        miner.deployRamp();
    }

    void update() override
    { 

    }

    bool isFinished() const override
    {
        return (millis() - startTime) > desiredTime;
    }

    void end() override
    {
        miner.stopMining();
        miner.store();
        shooter.stopFiring();
    }

    void configure(float desiredSeconds){
        desiredTime = desiredSeconds;
    }

private:
    Miner &miner;
    Shooter &shooter;
    unsigned long startTime = 0;
    float desiredTime; ///< Time to shoot for, in milliseconds
    bool hitsMet = false;
};