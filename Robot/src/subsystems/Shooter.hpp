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
        POSITION,
        VELOCITY,
        OFF,
        TEST
    };

    Shooter(int encoderA, int encoderB, TB9051Pins pins, float analog_vref = 5.0,
            MotorController::DriverType driverType = MotorController::DriverType::TB9051) : 
                _mode(OFF),
                motor(pins, SHOOTER_VELOCITY_PID, false),
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
        position = encoder.getCount() / (64.0 * SHOOTER_MOTOR_RATIO);
        velocity = encoder.getVelocity() / (64.0 * SHOOTER_MOTOR_RATIO); // encoder gives ticks/sec for velocity
        // Serial.print("Velocity ");
        // Serial.print(velocity);

        switch(_mode){
            case OFF:
                motor.setPower(0);
                break;
            case TEST:
                motor.setPower(255);
                break;
            case POSITION:
                motor.update(position);
                break;
            case VELOCITY:
                motor.update(velocity);
                break;
        };
        // motor.setSpeed((int)(targetVelocity * 10));
        
    }

    void fire()
    {
        // begin mining immediately on next update
        if (_mode != VELOCITY)
        {
            _mode = VELOCITY;
            motor.setPID(SHOOTER_VELOCITY_PID);
        }
        // Serial.println("setting speed to 255");
        targetVelocity = 1.5;
        motor.setTarget(targetVelocity);
    }

    void fire(float velocity)
    {
        // begin mining immediately on next update
        if (_mode != VELOCITY)
        {
            _mode = VELOCITY;
        }
        // Serial.println("setting speed to 255");
        targetVelocity = velocity;
    }

    void holdPosition( float position ){

        if (_mode != POSITION)
        {
            _mode = POSITION;
            motor.setPID(SHOOTER_POSITION_PID);
        }

        targetPosition = position;
        motor.setTarget(targetPosition);
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

    void hold(){
        holdPosition(position);
    }

private:
    Mode _mode = OFF;

    DualMotorController motor;
    EncoderWrapper encoder;
    unsigned long _cycleStartTime = 0;
    unsigned long _onStartTime = 0;
    float targetVelocity = 0;
    float targetPosition = 0;
    float position = 0;
    float velocity = 0;

};
