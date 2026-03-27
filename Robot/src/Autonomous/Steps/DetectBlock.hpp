/**
 * @file DetectBlock.hpp
 * @brief `AutoStep` that samples the color sensor and updates `Strategy` with the detected block type.
 *
 * Runs for `_checkTimeMs` milliseconds while polling the color sensor.
 * On expiry the step marks itself finished.
 */
#ifndef DETECT_BLOCK_STEP_H
#define DETECT_BLOCK_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"
#include "utils/Strategy.hpp"

/**
 * @brief Poll the color sensor for a configurable window, then finish.
 */
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

    /**
     * @brief Set the sampling window duration.
     * @param time  Duration to poll the sensor (ms).
     */
    void configure(unsigned int time){
        _checkTimeMs = time;
    }

private:
    unsigned int _checkTimeMs = 100; ///< Color-sensor sampling window (ms)
    unsigned long _startTime = 0;    ///< millis() at step start
    bool _isFinished = false;        ///< True after the sampling window expires

};

#endif