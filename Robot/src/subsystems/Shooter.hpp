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
                motor(pins, SHOOTER_POSITION_PID, false),
                encoder(encoderA, encoderB)
    {
    }
    

    /**
     * initializes shooter subsystem 
     */
    void init() override
    {
        // Initialize servo to retracted position and reset timers
        encoder.flipDirection();
        _cycleStartTime = 0;
        _onStartTime = 0;
        _mode = OFF;
        motor.init();
        motor.setPIDKpFunction(shooterFF);
        Serial.println("Shooter initialized");
        // Set to retract angle on startup
    }


    /**
     * Updates shooter subsystem:
     * updates children
     *  - encoder position/velocity
     * 
     * sets power to motor, depending on mode
     */
    void update() override
    {
        encoder.update();
        position = encoder.getCount()  * SHOOTER_TICKS_TO_ROTATIONS;
        velocity = encoder.getVelocity()  * SHOOTER_TICKS_TO_ROTATIONS; // encoder gives ticks/sec for velocity
        acceleration = encoder.getAcceleration()  * SHOOTER_TICKS_TO_ROTATIONS;
        // Serial.print("Velocity ");
        // Serial.print(velocity);

        switch(_mode){
            case OFF:
                motor.setPower(0);
                break;
            case TEST:
                motor.setPower(-130);
                break;
            case POSITION:
                motor.update(position, velocity);
                break;
            case VELOCITY:
                motor.update(velocity, acceleration);
                break;
        };
        // motor.setSpeed((int)(targetVelocity * 10));
        
    }


    /**
     * Move the motor to the next deadband position, in the positive direction
     */
    void fire()
    {
        if (_mode != POSITION)
        {
            _mode = POSITION;
            motor.resetPID();
            motor.setPID(SHOOTER_POSITION_PID);
        }
        targetPosition = ceil(targetPosition) + .12;
        motor.setTarget(targetPosition);
    }


    /** Set the speed of the shooter to a specific velocity to shoot */
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

    /** Set the shooter to hold a specific position */
    void holdPosition( float amount )
    {
        if (_mode != POSITION)
        {
            _mode = POSITION;
            motor.resetPID();
            motor.setPID(SHOOTER_POSITION_PID);
        }

        targetPosition = floor(targetPosition) + amount;
        motor.setTarget(targetPosition);
    }

    /** Command a power to the shooter motor */
    void fireHardSet(int signal){
        _mode = TEST;
    }

    void prime(){
        _mode = POSITION;
        if (_mode != POSITION)
        {
            _mode = POSITION;
            motor.resetPID();
            motor.setPID(SHOOTER_POSITION_PID);
        }

        //converting position to an integer using floor ensures the shooter will move in the positive direction
        //This has the assumption that the motor is in the deadband when the command is sent
        //Ensures shooter will not move in negative direction when rack is forward
        targetPosition = floor(targetPosition) + SHOOTER_PULL_BACK_ROTATIONS; //.1 is a safety factor in the event the shooter is slightly below an integer position. 
        motor.setTarget(targetPosition);
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

    /**Hold the current position of the shooter */
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
    float acceleration = 0;

};
