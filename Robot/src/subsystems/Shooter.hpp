#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"
#include "Devices/MotorController.hpp"
#include "Devices/Button.hpp"

class Shooter : public Subsystem
{
public:
    enum Mode
    {
        POSITION,
        VELOCITY,
        OFF,
        TEST,
        AUTO
    };

    enum AUTOFIRE_STATE{
        WAITFORBLOCK,
        HASBLOCK,
        FIRE
    };

    Shooter(
        uint16_t block_switch_pin, 
        uint16_t encoderA, 
        uint16_t encoderB, 
        TB9051Pins pins, 
        float analog_vref = 5.0,
        MotorController::DriverType driverType = MotorController::DriverType::TB9051) : 
                _mode(OFF),
                block_switch(block_switch_pin),
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
        block_switch.init();
        block_switch.reverse();
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
        // acceleration = encoder.getAcceleration()  * SHOOTER_TICKS_TO_ROTATIONS;
        // Serial.print("Velocity ");
        // Serial.print(velocity);
        // block_switch.update();
        // Serial.print(">SwitchValue:");
        // Serial.print(block_switch.getReading());
        // Serial.println("\r");

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
            case AUTO:
                handleAutoFire();
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
        targetPosition = nextFirePosition();
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

    /**
     * Have the motor go to the prime position. This will enable close-loop position control
     */
    void prime(){
        if (_mode != POSITION)
        {
            _mode = POSITION;
            motor.resetPID();
            motor.setPID(SHOOTER_POSITION_PID);
        }

        targetPosition = nextPrimePosition();
        motor.setTarget(targetPosition);
    }

    /**
     * Turns off the shooter motor.
     */
    void stopFiring()
    {
        _mode = OFF;
        targetVelocity = 0;
    }

    /**
     * Fire the shooter if there is a block detected
     * Otherwise, keep the shooter primed. 
     */
    void autoFire(){
        if (_mode != AUTO)
        {
            _mode = AUTO;
            motor.resetPID();
            motor.setPID(SHOOTER_POSITION_PID);
        }

        //Prime the shooter
        targetPosition = floor(targetPosition) + SHOOTER_PULL_BACK_ROTATIONS; //.1 is a safety factor in the event the shooter is slightly below an integer position.
        motor.setTarget(targetPosition);
    }


    /**
     * Inherited from Subsystem. This does the same thing as stopFiring()
     */
    void stop() override
    {
        _mode = OFF;
        targetVelocity = 0;
    }

    /**
     * Hold the current position of the shooter. This can be used as a "pause"
     * */
    void hold(){
        holdPosition(position);
    }

private:


    /**
     * State machine handling for autofire mode
     */
    void handleAutoFire()
    {
        unsigned long now = millis();
        switch( autoFire_state){
            case WAITFORBLOCK:
            {
                // Detect rising edge of switch
                block_switch.update();
                bool reading = block_switch.getReading();
                if (reading && reading != last_switch_value)
                {
                    _blockDetectedTime = now;
                    autoFire_state = HASBLOCK;
                }
                break;
            }
            
            case HASBLOCK:
            {
                if( now - _blockDetectedTime > SHOOTER_SETTLE_TIME){
                    //After enough time has passed for block to settle, fire the shooter
                    targetPosition = nextFirePosition();
                    motor.setTarget(targetPosition);
                    autoFire_state = FIRE;
                    _fireTime = now;
                }
                break;
            }
            
            case FIRE:
            {
                //Allow rack to settle before firing again
                if( now - _fireTime > SHOOTER_FIRE_TIME ){
                    targetPosition = nextPrimePosition();
                    motor.setTarget(targetPosition);
                    //Set shooter to wait for block state;
                    autoFire_state = WAITFORBLOCK;
                }
                break;
            }

        }
        //Update motor controller regardless of state
        motor.update(position, velocity);
    }


    /**
     * Gets the next position for the encoder to go to to prime the shooter
     */
    float nextPrimePosition( ){
        // converting position to an integer using floor ensures the shooter will move in the positive direction
        // This has the assumption that the motor is in the deadband when the command is sent
        // Ensures shooter will not move in negative direction when rack is forward
        return floor(targetPosition) + SHOOTER_PULL_BACK_ROTATIONS; //.1 is a safety factor in the event the shooter is slightly below an integer position.
    }

    /**
     * Gets the position to got to in order to fire the shooter
     */
    float nextFirePosition(){
        return ceil(targetPosition) + .12;
    }

    Mode _mode = OFF;
    AUTOFIRE_STATE autoFire_state = WAITFORBLOCK; 

    Button block_switch;
    DualMotorController motor;
    EncoderWrapper encoder;


    bool last_switch_value = 0;
    unsigned long _fireTime = 0;
    unsigned long _blockDetectedTime = 0;
    unsigned long _cycleStartTime = 0;
    unsigned long _onStartTime = 0;
    float targetVelocity = 0;
    float targetPosition = 0;
    float position = 0;
    float velocity = 0;
    float acceleration = 0;

};
