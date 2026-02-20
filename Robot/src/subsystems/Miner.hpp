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
    Miner(const int servoPin)
        : servo(servoPin, 0, 180, false)
    {
    }

    enum Mode
    {
        MINING,
        OFF,
        STORE
    };

    // Sentinel value for "mine indefinitely"
    static constexpr int32_t INDEFINITE = -1;

    void init() override
    {
        _cycleStartTime = 0;
        _onStartTime = 0;
        _mode = STORE;
        _number_hits = 0;
        _miningStartTime = 0;
        _timedOut = false;

        servo.init();
        servo.setAngle(MINER_SERVO_STORE_ANGLE);
        Serial.println("Miner initialized");
    }

    void update() override
    {
        const unsigned long now = millis();

        // If programmed to stop after a fixed number of hits, and we've reached that,
        // stop mining. Note: compare only if _hits_to_mine is not INDEFINITE.
        if (_hits_to_mine != INDEFINITE && _number_hits >= static_cast<uint32_t>(_hits_to_mine))
        {
            stopMiningInternal();
        }

        // If we timed out previously, keep miner OFF until explicitly restarted
        if (_timedOut)
        {
            // ensure servo retracted and don't auto-reset timers
            setServoToRetract();
            servo.update();
            return;
        }

        if (_mode == STORE)
        {
            // Ensure servo is retracted while stopped and timers cleared for next start
            setServoToStore();
            _cycleStartTime = 0;
            _onStartTime = 0;
            servo.update();
            return;
        }

        if (_mode == OFF)
        {
            // Ensure servo is retracted while stopped and timers cleared for next start
            setServoToRetract();
            _cycleStartTime = 0;
            _onStartTime = 0;
            servo.update();
            return;
        }

        // MINING mode ---------------------------------------------------------
        // Start global mining timer on first transition to MINING
        if (_miningStartTime == 0)
            _miningStartTime = now;

        // Safety: if mining indefinitely but exceeded configured maximum continuous time, stop.
        if (_hits_to_mine == INDEFINITE && _maxContinuousMillis > 0 &&
            (now - _miningStartTime) >= _maxContinuousMillis)
        {
            // mark timed out and stop
            _timedOut = true;
            stopMiningInternal();
            Serial.println("Miner timed out (max continuous time exceeded)");
            servo.update();
            return;
        }

        const unsigned long cycleMs = MINER_CYCLE_MS;
        const unsigned long pressMs = MINER_PRESS_MS;

        // Start a new cycle if needed
        if (_cycleStartTime == 0)
        {
            _cycleStartTime = now;
            _onStartTime = now; // start pressing immediately at cycle start
            setServoToPress();
            servo.update();
            return;
        }

        // How long since the cycleStart (mod cycle length) — keeps values bounded
        unsigned long elapsedSinceStart = now - _cycleStartTime;
        unsigned long elapsedInCycle = elapsedSinceStart % cycleMs;

        // If within press window -> ensure pressing
        if (elapsedInCycle < pressMs)
        {
            // if we just entered press window (no onStart recorded or long past), record
            if (_onStartTime == 0 || (now - _onStartTime) > cycleMs)
            {
                _onStartTime = now;
                setServoToPress();
            }
            // otherwise keep pressing (no-op)
        }
        else
        {
            // outside press window -> ensure retracted and reset onStart
            _onStartTime = 0;
            setServoToRetract();

            // If one or more full cycles have elapsed since _cycleStartTime, advance and count hits
            if (elapsedSinceStart >= cycleMs)
            {
                // How many cycles have passed fully since _cycleStartTime
                unsigned long cyclesPassed = elapsedSinceStart / cycleMs;

                // Prevent extremely large jumps (protect against very long pauses)
                const unsigned long MAX_CYCLES_INCREMENT = 10000UL;
                if (cyclesPassed > MAX_CYCLES_INCREMENT)
                    cyclesPassed = MAX_CYCLES_INCREMENT;

                _cycleStartTime += cyclesPassed * cycleMs;

                // Increase hit count by the number of completed cycles
                _number_hits += static_cast<uint32_t>(cyclesPassed);

                // If we reached / exceeded a finite goal, clamp and stop
                if (_hits_to_mine != INDEFINITE && _number_hits >= static_cast<uint32_t>(_hits_to_mine))
                {
                    stopMiningInternal();
                }
            }
        }

        // Always update servo at end so PWM/system-level updates occur
        servo.update();
    }

    // Start mining with a specific number of hits (finite)
    void mine(int32_t hits_to_mine)
    {
        if (hits_to_mine < 0)
            hits_to_mine = INDEFINITE;

        _hits_to_mine = hits_to_mine;
        startMining();
    }

    // Start indefinite mining (preferred over magic numbers)
    void mineIndefinitely()
    {
        _hits_to_mine = INDEFINITE;
        startMining();
    }

    void stopMining()
    {
        stopMiningInternal();
    }

    void stop() override
    {
        stopMiningInternal();
    }

    bool isDoneMining() const
    {
        return (_mode == OFF);
    }

    void store()
    {
       setServoToStore(); 
    }

    // Return true if miner auto-stopped because of timeout (only relevant for indefinite)
    bool hasTimedOut() const { return _timedOut; }

    // Accessors
    uint32_t getHitsDone() const { return _number_hits; }
    int32_t getHitsTarget() const { return _hits_to_mine; }

    // Configure safety timeout for indefinite mining: 0 = disabled.
    // Default is 5 minutes (300000 ms). Call setMaxContinuousMillis(0) to disable.
    void setMaxContinuousMillis(unsigned long ms) { _maxContinuousMillis = ms; }

    // Reset timeout state (allow restart after a timeout)
    void clearTimeout()
    {
        _timedOut = false;
        _miningStartTime = 0;
    }

private:
    Mode _mode = OFF;
    ServoControl servo;
    unsigned long _cycleStartTime = 0; // start of current cycle anchor
    unsigned long _onStartTime = 0;    // when press window started
    uint32_t _number_hits = 0;         // how many cycles completed (unsigned)
    int32_t _hits_to_mine = 10;        // target hits; INDEFINITE (-1) means run forever

    // Safety: guard against indefinite mining running forever by specifying a max duration
    unsigned long _maxContinuousMillis = 300000UL; // default 5 minutes
    unsigned long _miningStartTime = 0;            // when mining() first called
    bool _timedOut = false;

    // Helper wrappers to set servo position
    void setServoToPress()
    {
        servo.setAngle(MINER_SERVO_PRESS_ANGLE);
    }

    void setServoToRetract()
    {
        servo.setAngle(MINER_SERVO_RETRACT_ANGLE);
    }

    void setServoToStore()
    {
        servo.setAngle(MINER_SERVO_STORE_ANGLE);
    }

    void startMining()
    {
        if (_mode != MINING)
        {
            _mode = MINING;
            _number_hits = 0;
            _cycleStartTime = 0;
            _onStartTime = 0;
            _miningStartTime = 0; // will be set on first update
            _timedOut = false;
        }
    }

    void stopMiningInternal()
    {
        _mode = OFF;
        _cycleStartTime = 0;
        _onStartTime = 0;
        _miningStartTime = 0;
        // leave _number_hits as-is so caller can read progress
        // ensure servo is retracted on next update (or we can retract immediately)
        setServoToRetract();
        servo.update();
    }
};
