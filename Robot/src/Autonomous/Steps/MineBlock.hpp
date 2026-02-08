#pragma once

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

class MineBlockStep : public AutoStep
{
public:
    MineBlockStep(){
        
    }

    void start() override
    {
    }

    void update() override
    { 
        //Create logic for pressing mine button here


    }

    bool isFinished() const override
    {
        return true;
    }

    void end() override
    {
    }

    void configure(int hits){
        _numberHits = hits;
    }

private:
    short _numberHits = 5;
    bool hitsMet = false;
};