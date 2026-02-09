#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"

class Miner : public Subsystem
{
public:
    Miner(const int servoPin) : servo(servoPin, 0, 180, false)
    {
    }

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
        servo.init();

        // Set to retract angle on startup
        servo.setAngle(MINER_SERVO_STORE_ANGLE);
        Serial.println("Miner initialized");
    }

    void update() override
    {
        // Use unsigned long for millis arithmetic
        const unsigned long now = millis();

        if( _number_hits >= _hits_to_mine - 1){
            _mode = OFF;
        }

        if (_mode == OFF)
        {
            // Ensure servo is retracted while stopped
            setServoToRetract();
            // reset cycle so next mine() starts fresh
            _cycleStartTime = 0;
            _onStartTime = 0;
            return;
        }

        // MINING mode
        const unsigned long cycleMs = MINER_CYCLE_MS;
        const unsigned long pressMs = MINER_PRESS_MS;

        // start a new cycle if needed
        if (_cycleStartTime == 0)
        {
            _cycleStartTime = now;
            // start with press immediately at cycle start
            _onStartTime = now;
            setServoToPress();
            return;
        }

        const unsigned long elapsedInCycle = (now - _cycleStartTime) % cycleMs;
        // If we're within the press window, ensure servo is pressing
        if (elapsedInCycle < pressMs)
        {
            // If we just entered press window, record on start and set press pos
            if (_onStartTime == 0 || (now - _onStartTime) > cycleMs)
            {
                _onStartTime = now;
                setServoToPress();
            }
            else
            {
                // keep pressing (no-op)
            }
        }
        else
        {
            // Press window elapsed, ensure retracted
            _onStartTime = 0;
            setServoToRetract();
            

            // If the full cycle has elapsed, advance cycle start (keeps times small)
            // This keeps cycle anchored to the first cycleStartTime but also avoids overflow
            if ((now - _cycleStartTime) >= cycleMs)
            {
                // advance cycle start forward by multiples of cycleMs to avoid long drift
                unsigned long cyclesPassed = (now - _cycleStartTime) / cycleMs;
                _cycleStartTime += cyclesPassed * cycleMs;
                _number_hits++;
            }
        }
        servo.update();
    }

    void mine()
    {
        // begin mining immediately on next update
        //indefinite number of hits
        _hits_to_mine = 100000;
        if (_mode != MINING)
        {
            _mode = MINING;
            // reset timers so the next update starts a clean cycle immediately
            _number_hits = 0;
            _cycleStartTime = 0;
            _onStartTime = 0;
        }
    }

    void mine(int hits_to_mine)
    {
        // begin mining immediately on next update
        _hits_to_mine = hits_to_mine;
        if (_mode != MINING)
        {
            _mode = MINING;
            // reset timers so the next update starts a clean cycle immediately
            _number_hits = 0;
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

    bool isDoneMining(){
        return (_mode ==  OFF);
    }

private:
    Mode _mode = OFF;
    ServoControl servo;
    unsigned long _cycleStartTime = 0;
    unsigned long _onStartTime = 0;
    unsigned int _number_hits = 0;
    unsigned int _hits_to_mine = 10;




    // Helper wrappers to set servo position. Replace servo.write(...) if your ServoControl uses another API.
    void setServoToPress()
    {
        // If your ServoControl uses a different function (e.g. setPosition, moveTo),
        // replace the next line with that call.
        servo.setAngle(MINER_SERVO_PRESS_ANGLE);
    }

    void setServoToRetract()
    {
        // If your ServoControl uses a different function (e.g. setPosition, moveTo),
        // replace the next line with that call.
        servo.setAngle(MINER_SERVO_RETRACT_ANGLE);
    }
};
