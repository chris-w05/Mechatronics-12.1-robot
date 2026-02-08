#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Devices/MotorController.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"

class Miner: public Subsystem
{
public:
    Miner(const int servoPin):
    servo(servoPin, 0, 180, false)
    {}


    enum Mode{
        MINING,
        OFF
    };

    void init() override
    {

    }

    void update() override
    {
        
    }

    void mine(){
        _mode = MINING;
    }

    void stopMining(){
        _mode = OFF;
    }

    void stop() override
    {
        _mode = OFF;
    }
    // Needs to be updated if using PID

private:
    Mode _mode = OFF;
    ServoControl servo;
    long _cycleStartTime = 0;
    long _onStartTime = 0;
    

};