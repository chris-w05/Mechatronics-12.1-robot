/**
 * @file Shooter.hpp
 * @brief Rack-and-pinion block shooter subsystem.
 *
 * The Shooter drives a DC motor through a rack-and-pinion mechanism to fire
 * foam blocks from the robot.  It supports four operating modes:
 *   - **POSITION** — closed-loop encoder position control (used for priming and firing).
 *   - **VELOCITY** — closed-loop encoder velocity control.
 *   - **AUTO**     — autofire state machine: waits for a block on `block_switch`,
 *                    settles, fires, then re-primes automatically.
 *   - **OFF/TEST** — open-loop (coast or fixed power).
 *
 * The motor is tuned with a nonlinear feedforward via `shooterFF()` from Config.hpp.
 */
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

/**
 * @brief Rack-and-pinion block shooter controlled by encoder feedback.
 *
 * Owns a `DualMotorController`, an `EncoderWrapper` for position/velocity
 * feedback, and a `Button` that detects when a block is seated in the chamber.
 */
class Shooter : public Subsystem
{
public:
    /** @brief Motor control mode selection. */
    enum Mode
    {
        POSITION,   ///< Closed-loop encoder position control
        VELOCITY,   ///< Closed-loop encoder velocity control
        OFF,        ///< Motor coasting (zero power)
        TEST,       ///< Fixed open-loop power for bench-testing
        AUTO        ///< Autonomous autofire state machine
    };

    /** @brief Sub-states of the autofire state machine (used in AUTO mode). */
    enum AUTOFIRE_STATE{
        WAITFORBLOCK,   ///< Idle — waiting for the block-present switch to trigger
        HASBLOCK,       ///< Block detected — waiting for the settle delay
        FIRE,           ///< Block settled — motor driving to fire position
    };

    /**
     * @brief Construct the Shooter subsystem.
     * @param block_switch_pin  Digital pin for the block-present switch.
     * @param encoderA          Encoder channel A pin.
     * @param encoderB          Encoder channel B pin.
     * @param pins              TB9051 motor driver pin struct.
     * @param analog_vref       ADC reference voltage (default 5.0 V, unused).
     * @param driverType        Motor driver variant (default TB9051).
     */
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
     * @brief Initialise the shooter subsystem.
     *
     * Resets the encoder direction, zeroes cycle timers, sets the motor PID
     * feedforward function, and places the subsystem in OFF mode.
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
     * @brief Update the shooter — must be called every control loop tick.
     *
     * Refreshes encoder position/velocity, then dispatches to the active mode
     * (OFF, TEST, POSITION, VELOCITY, or AUTO state machine).
     */
    void update() override
    {
        now = millis();
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
     * @brief Fire one shot by advancing the rack to the next fire position.
     *
     * Switches to POSITION mode and commands the motor to `nextFirePosition()`.
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


    /**
     * @brief Fire at a specific motor velocity (VELOCITY mode).
     * @param velocity  Target rotational velocity in rotations/s.
     */
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

    /**
     * @brief Command a fractional position offset from the current integer position.
     * @param amount  Fractional rotations to add to `floor(targetPosition)`.
     */
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

    /**
     * @brief Enable TEST mode (fixed open-loop power) for bench testing.
     * @param signal  (Unused in current implementation — mode is set to TEST.)
     */
    void fireHardSet(int signal){
        _mode = TEST;
    }

    /**
     * @brief Pull the rack back to the primed (loaded) position using POSITION control.
     *
     * Commands the motor to `nextPrimePosition()`, which is `SHOOTER_PRIME_POSITION`
     * above the current integer encoder position.
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
     * @brief Stop the shooter motor and clear the velocity target.
     */
    void stopFiring()
    {
        _mode = OFF;
        targetVelocity = 0;
    }

    /**
     * @brief Enable AUTO mode: fire automatically whenever `block_switch` detects a block.
     *
     * Enters the AUTOFIRE state machine which cycles through
     * WAITFORBLOCK → HASBLOCK (settle delay) → FIRE → WAITFORBLOCK.
     */
    void autoFire(){
        if (_mode != AUTO)
        {
            _mode = AUTO;
            motor.resetPID();
            motor.setPID(SHOOTER_POSITION_PID);
            _fireTime = millis();
        }

        //Prime the shooter
        targetPosition = floor(targetPosition) + SHOOTER_PRIME_POSITION; //.1 is a safety factor in the event the shooter is slightly below an integer position.
        motor.setTarget(targetPosition);
    }


    /**
     * @brief Stop the shooter (Subsystem interface — identical to stopFiring()).
     */
    void stop() override
    {
        _mode = OFF;
        targetVelocity = 0;
    }

    /**
     * @brief Freeze the shooter at its current encoder position (POSITION mode hold).
     */
    void hold(){
        holdPosition(position);
    }

private:


    /** @brief Run one tick of the autofire state machine (called inside update()). */
    void handleAutoFire()
    {
        switch( autoFire_state){
            case WAITFORBLOCK:
            {
                // Detect rising edge of switch
                block_switch.update();
                bool is_block_present = block_switch.getReading();

                //Start fire procedure if there is a block
                if (is_block_present)
                {
                    _blockDetectedTime = now;
                    autoFire_state = HASBLOCK;
                }

                //Jump straight to shooting if there has been a suspiciously long time without a block
                if (now - _fireTime > SHOOTER_JAM_DETECT_TIME)
                {
                    targetPosition = nextFirePosition();
                    motor.setTarget(targetPosition);
                    autoFire_state = FIRE;
                    _fireTime = now;
                }
                break;
            }
            
            //Do nothing until the block settles
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
     * @brief Compute the encoder target for the primed (pulled-back) position.
     * @return Target position in rotations.
     */
    const float nextPrimePosition( ){
        // Moves shooter in positive direction to the nearest n + SHOOTER_PRIME_POSITION without passing SHOOTER_FIRE_POSITION
        // 1e-6f prevents floating point error from causing errors on repeated prime calls. 
        return floor(targetPosition + (1 - SHOOTER_PRIME_POSITION) - 1e-6f) + SHOOTER_PRIME_POSITION;
    }


    /**
     * @brief Compute the encoder target for the fire (extended) position.
     * @return Target position in rotations.
     */
    const float nextFirePosition(){
        //Hardcoded 1e-6 prevents floating point errors when Fire() is repeatedly called
        return floor(targetPosition + (1 - SHOOTER_FIRE_POSITION) + 1e-6f) + SHOOTER_FIRE_POSITION;
    }

    Mode _mode = OFF;                              ///< Current operating mode
    AUTOFIRE_STATE autoFire_state = WAITFORBLOCK;  ///< Current state of the autofire state machine

    Button block_switch;        ///< Switch that detects a seated block in the chamber
    DualMotorController motor;  ///< PID-controlled dual motor driver for the rack
    EncoderWrapper encoder;     ///< Quadrature encoder providing position and velocity

    unsigned long now = 0;                  ///< current time in milliseconds
    unsigned long _fireTime = 0;            ///< millis() when the fire command was issued
    unsigned long _blockDetectedTime = 0;   ///< millis() when the block-present switch triggered
    unsigned long _cycleStartTime = 0;      ///< millis() at the start of the current fire cycle
    unsigned long _onStartTime = 0;         ///< millis() when the motor was first energised this cycle
    float targetVelocity = 0;              ///< Commanded velocity setpoint (rotations/s)
    float targetPosition = 0;              ///< Commanded position setpoint (rotations)
    float position = 0;                    ///< Current encoder position (rotations)
    float velocity = 0;                    ///< Current encoder velocity (rotations/s)
    float acceleration = 0;               ///< Current encoder acceleration (rotations/s²)

};
