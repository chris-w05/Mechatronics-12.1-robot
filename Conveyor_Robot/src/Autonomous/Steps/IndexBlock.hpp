#ifndef PLACE_BLOCK_STEP_H
#define PLACE_BLOCK_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "Devices/Encoder.hpp"
#include "utils/Strategy.hpp"

class IndexBlockStep : public AutoStep
{
public:
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

    void configure(Strategy::IndexDirection dir){
        _dir = dir;
    }

private:
    Strategy::IndexDirection _dir;
};

#endif