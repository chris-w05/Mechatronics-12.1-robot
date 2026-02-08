#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"
#include "Devices/SingleMotorController.hpp"

class Shooter : public Subsystem
{
public:
    Shooter(const int servoPin);

    enum Mode
    {
        MINING,
        OFF
    };

    void init() override
    {
        // Initialize servo to retracted position and reset timers
        _cycleStartTime = 0;
        _onStartTime = 0;
        _mode = OFF;

        // Set to retract angle on startup
    }

    void update() override
    {
        
    }

    void mine()
    {
        // begin mining immediately on next update
        if (_mode != MINING)
        {
            _mode = MINING;
            // reset timers so the next update starts a clean cycle immediately
            _cycleStartTime = 0;
            _onStartTime = 0;
        }
    }

    void stopMining()
    {
        _mode = OFF;
    }

    void stop() override
    {
        _mode = OFF;
    }

private:
    Mode _mode = OFF;
    SingleMotorController motor;
    unsigned long _cycleStartTime = 0;
    unsigned long _onStartTime = 0;

};
