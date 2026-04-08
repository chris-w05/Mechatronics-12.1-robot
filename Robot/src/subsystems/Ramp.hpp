#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"

class Ramp : public Subsystem
{
public:
    Ramp(const int servoPin)
        : servo(servoPin, 0, 180, false)
    {
    }

    enum Mode
    {
        STORE,
        IDLE,
        PASSIVE
    };

    void init() override
    {
        _mode = STORE;
//        _timedOut = false;

        servo.init();
        servo.setAngle(RAMP_SERVO_STORE_ANGLE);
        Serial.println("Ramp initialized");
    }

    void update() override
    {
        _now = millis();

        // If we timed out previously, keep ramp PASSIVE until explicitly restarted
//        if (_timedOut)
//        {
//            // ensure servo retracted and don't auto-reset timers
//            setServoToPassive();
//            servo.update();
//            return;
//        }

        switch(_mode){

            case PASSIVE:
                // Ensure servo is retracted while stopped and timers cleared for next start
                setServoToPassive();
                servo.update();
                return;
            
            case STORE:
                setServoToStore();
                servo.update();
                return;

            case IDLE:
                setServoToIdle();
                servo.update();
                return;
        }

        
    }

   // Return true if miner auto-stopped because of timeout (only relevant for indefinite)
//    bool hasTimedOut() const { return _timedOut; }

    // Configure safety timeout for indefinite mining: 0 = disabled.
    // Default is 5 minutes (300000 ms). Call setMaxContinuousMillis(0) to disable.
//    void setMaxContinuousMillis(unsigned long ms) { _maxContinuousMillis = ms; }

    // Reset timeout state (allow restart after a timeout)
//    void clearTimeout()
//    {
//        _timedOut = false;
//    }
//
private:
    Mode _mode = PASSIVE;
    ServoControl servo;
    /** Current time */
    unsigned long _now = 0;

    /**
     * Change the posiiton of the miner to store the ramp
     */
    void setServoToStore()
    {
        servo.setAngle(RAMP_SERVO_STORE_ANGLE);
    }

    /**
     * Change the posiiton of the miner to store the ramp
     */
    void setServoToIdle()
    {
        servo.setAngle(RAMP_SERVO_STORE_ANGLE);
    }

    /**
     * Change the posiiton of the ramp to passive
     */
    void setServoToPassive()
    {
        servo.setAngle(RAMP_SERVO_PASSIVE_ANGLE);
    }
};
