/**
 * @file MineBlockAutofire.hpp
 * @brief `AutoStep` that mines and simultaneously autofires the shooter for a time window.
 *
 * `start()` engages indefinite mining, deploys the ramp, and starts the
 * shooter's autofire mode.  `end()` stops both subsystems and stows the ramp.
 */
#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/Shooter.hpp"

/**
 * @brief Runs the miner and shooter autofire concurrently for `desiredSeconds` seconds.
 */
class MineBlockAutofire : public AutoStep
{
public:
    /**
     * @brief Construct the step.
     * @param miner        Miner subsystem reference.
     * @param shooter      Shooter subsystem reference.
     * @param timeSeconds  Duration to run mining + autofire (seconds).
     */
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

    /**
     * @brief Re-configure the run duration for reuse.
     * @param desiredSeconds  New duration (seconds).
     */
    void configure(float desiredSeconds){
        desiredTime = desiredSeconds;
    }

private:
    Miner &miner;              ///< Miner subsystem reference
    Shooter &shooter;          ///< Shooter subsystem reference
    unsigned long startTime = 0; ///< millis() at step start
    float desiredTime;         ///< Run duration converted to milliseconds
    bool hitsMet = false;      ///< (unused)
};