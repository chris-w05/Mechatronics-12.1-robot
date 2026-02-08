#ifndef DETECT_BLOCK_STEP_H
#define DETECT_BLOCK_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "utils/Strategy.hpp"

class DetectBlockStep : public AutoStep
{
public:
    DetectBlockStep(){
        
    }

    void start() override
    {
        _startTime = millis();
        _isFinished = false;
    }

    void update() override
    {

        if( millis() - _startTime > _checkTimeMs){
            _isFinished = true;
            return;
        }

        //Read a color sensor's value, then update Strategy with the new block
    }

    bool isFinished() const override
    {
        return _isFinished;
    }

    void end() override
    {
    }

    void configure(unsigned int time){
        _checkTimeMs = time;
    }

private:
    unsigned int _checkTimeMs = 100;
    unsigned long _startTime = 0;
    bool _isFinished = false;

};

#endif