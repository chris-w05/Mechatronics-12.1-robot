#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/Encoder.hpp"
#include "utils/PID.hpp"
#include "Devices/MotorController.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"

class Miner: public Subsystem
{
public:
    Miner(const int solenoidPin):
    mineSolenoid(solenoidPin)
    {}


    enum Mode{
        MINING,
        OFF
    };

    void init() override
    {
        mineSolenoid.init();
    }

    void update() override
    {
        unsigned long now = millis();

        // Start a new cycle
        if (!mineSolenoid.isOn() && now - _cycleStartTime >= MINER_HIT_RATE)
        {
            _cycleStartTime = now;
            _onStartTime = now;
            mineSolenoid.on();
        }

        // Turn off after max on time
        if (mineSolenoid.isOn() && now - _onStartTime >= SOLENOID_MAX_ON_TIME)
        {
            mineSolenoid.off();
        }
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
        mineSolenoid.off();
    }
    // Needs to be updated if using PID

private:
    Mode _mode = OFF;
    Solenoid mineSolenoid;
    long _cycleStartTime = 0;
    long _onStartTime = 0;
    

};