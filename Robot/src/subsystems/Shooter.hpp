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
        OFF,
        TEST
    };

    Shooter(int encoderA, int encoderB, TB9051Pins pins, float analog_vref = 5.0,
            MotorController::DriverType driverType = MotorController::DriverType::TB9051) : 
                _mode(OFF),
                motor(pins, SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, false),
                encoder(encoderA, encoderB)
    {
    }



    

    void init() override
    {
        // Initialize servo to retracted position and reset timers
        _cycleStartTime = 0;
        _onStartTime = 0;
        _mode = OFF;
        Serial.println("Shooter initialized");
        // Set to retract angle on startup
    }

    void update() override
    {
        encoder.update();
        float velocity = encoder.getVelocity() / (64.0 * SHOOTER_MOTOR_RATIO); //encoder gives ticks/sec for velocity
        // Serial.print("Velocity ");
        // Serial.print(velocity);
        motor.update(targetVelocity, velocity);
        if( _mode == OFF){
            motor.setPower(0);
        }
        if (_mode == TEST){
            motor.setPower(255);
        }
        // motor.setSpeed((int)(targetVelocity * 10));
        
    }

    void fire()
    {
        // begin mining immediately on next update
        if (_mode != MINING)
        {
            _mode = MINING;
        }
        // Serial.println("setting speed to 255");
        targetVelocity = 1.5;
    }

    void fire(float velocity)
    {
        // begin mining immediately on next update
        if (_mode != MINING)
        {
            _mode = MINING;
        }
        // Serial.println("setting speed to 255");
        targetVelocity = velocity;
    }

    void fireHardSet(int signal){
        _mode = TEST;
    }

    void stopFiring()
    {
        _mode = OFF;
        targetVelocity = 0;
    }

    void stop() override
    {
        _mode = OFF;
        targetVelocity = 0;
    }

private:
    Mode _mode = OFF;
    // MotorController motor;
    DualMotorController motor;
    EncoderWrapper encoder;
    unsigned long _cycleStartTime = 0;
    unsigned long _onStartTime = 0;
    float targetVelocity = 0;

};
