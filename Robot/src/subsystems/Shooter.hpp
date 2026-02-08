#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"
#include "Devices/MotorController.hpp"

class Shooter : public Subsystem
{
public:
    enum Mode
    {
        MINING,
        OFF
    };

    Shooter(int pwm1_pin, int pwm2_pin = -1, int en_pin = -1, int enb_pin = -1,
            int diag_pin = -1, int ocm_pin = -1, int occ_pin = -1, float analog_vref = 5.0,
            MotorController::DriverType driverType = MotorController::DriverType::L298N) : 
                _mode(OFF),
                motor(pwm1_pin, pwm2_pin, en_pin, enb_pin, diag_pin, ocm_pin, occ_pin, analog_vref, 0, 0, 0, driverType)
    {
    }



    

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

    void fire()
    {
        // begin mining immediately on next update
        if (_mode != MINING)
        {
            _mode = MINING;
        }
        motor.setSpeed(255);
    }

    void stopFiring()
    {
        _mode = OFF;
        motor.setSpeed(0);
    }

    void stop() override
    {
        _mode = OFF;
    }

private:
    Mode _mode = OFF;
    MotorController motor;
    unsigned long _cycleStartTime = 0;
    unsigned long _onStartTime = 0;

};
