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
    Miner(const int minerServoPin, const int rampServoPin)
        : minerServo(minerServoPin, 0, 180, false),
          rampServo(rampServoPin, 0, 180, false)
    {
    }

    enum Mode
    {
        LIFT,
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

        minerServo.init();
        rampServo.init();
        setServosToStore();
        minerServo.update();
        rampServo.update();
        Serial.println("Miner initialized");
    }

    void update() override
    {
        _now = millis();

        // If programmed to stop after a fixed number of hits, and we've reached that,
        // stop mining. Note: compare only if _hits_to_mine is not INDEFINITE.
        if (_hits_to_mine != INDEFINITE && _number_hits >= static_cast<uint32_t>(_hits_to_mine))
        {
            stopMiningInternal();
        }

        // If we timed out previously, keep miner OFF until explicitly restarted
        if (_timedOut)
        {
            // ensure minerServo retracted and don't auto-reset timers
            setMinerServoToRetract();
            minerServo.update();
            return;
        }

        switch(_mode){

            case LIFT:
                setRampServoToLift();
                rampServo.update();
                return;

            case STORE:
                // Ensure minerServo is retracted while stopped and timers cleared for next start
                setServosToStore();
                _cycleStartTime = 0;
                _onStartTime = 0;
                minerServo.update();
                rampServo.update();
                return;
            
            case OFF:
                // Ensure minerServo is retracted while stopped and timers cleared for next start
                setMinerServoToRetract();
                _cycleStartTime = 0;
                _onStartTime = 0;
                minerServo.update();
                return;
            
            case MINING:
                mine();
                setRampServoToPassive();
                rampServo.update();
                return;
        }

        
    }


    /**
     *  Start mining with a specific number of hits (finite)
     * @param hits_to_mine Number of hits before the miner stops mining
     * */
    void startMining(int32_t hits_to_mine)
    {
        if (hits_to_mine < 0)
            hits_to_mine = INDEFINITE;

        _hits_to_mine = hits_to_mine;
        startMining();
    }

    // Start indefinite mining (preferred over magic numbers)
    void startMiningIndefinitely()
    {
        _hits_to_mine = INDEFINITE;
        startMining();
    }

    void stopMining()
    {
        stopMiningInternal();
    }

    void deployRamp(){
        setRampServoToPassive();
    }

    void stop() override
    {
        stopMiningInternal();
        setServosToStore();
    }

    bool isDoneMining() const
    {
        return (_mode == OFF);
    }

    /**
     * Number of hits must be 0 for the miner to move from the retracted state

     * @param _number_hits = 0
     */
    void store()
    {
        _number_hits = 0; // Miner cannot move from retract unless _number_hits is 0
        _mode = STORE;
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
    ServoControl minerServo;
    ServoControl rampServo;
    /** Current time */
    unsigned long _now = 0;
    /**Start of current cycle */
    unsigned long _cycleStartTime = 0;
    /**Start of press window */
    unsigned long _onStartTime = 0;
    uint32_t _number_hits = 0;         // how many cycles completed (unsigned)
    int32_t _hits_to_mine = 10;        // target hits; INDEFINITE (-1) means run forever

    // Safety: guard against indefinite mining running forever by specifying a max duration
    unsigned long _maxContinuousMillis = 300000UL; // default 5 minutes
    unsigned long _miningStartTime = 0;            // when mining() first called
    bool _timedOut = false;

    void mine(){
        // MINING mode ---------------------------------------------------------
        // Start global mining timer on first transition to MINING
        if (_miningStartTime == 0)
            _miningStartTime = _now;

        // Safety: if mining indefinitely but exceeded configured maximum continuous time, stop.
        if (_hits_to_mine == INDEFINITE && _maxContinuousMillis > 0 &&
            (_now - _miningStartTime) >= _maxContinuousMillis)
        {
            // mark timed out and stop
            _timedOut = true;
            stopMiningInternal();
            Serial.println("Miner timed out (max continuous time exceeded)");
            minerServo.update();
            return;
        }

        const unsigned long cycleMs = MINER_CYCLE_MS;
        const unsigned long pressMs = MINER_PRESS_MS;

        // Start a new cycle if needed
        if (_cycleStartTime == 0)
        {
            _cycleStartTime = _now;
            _onStartTime = _now; // start pressing immediately at cycle start
            setMinerServoToPress();
            minerServo.update();
            return;
        }

        // How long since the cycleStart (mod cycle length) — keeps values bounded
        unsigned long elapsedSinceStart = _now - _cycleStartTime;
        unsigned long elapsedInCycle = elapsedSinceStart % cycleMs;

        // If within press window -> ensure pressing
        if (elapsedInCycle < pressMs)
        {
            // if we just entered press window (no onStart recorded or long past), record
            if (_onStartTime == 0 || (_now - _onStartTime) > cycleMs)
            {
                _onStartTime = _now;
                setMinerServoToPress();
            }
            // otherwise keep pressing (no-op)
        }
        else
        {
            // outside press window -> ensure retracted and reset onStart
            _onStartTime = 0;
            setMinerServoToRetract();

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
//                if ((_number_hits % 10) == 0)
//                {
//                    setServoToIdle();
//                    rampServo.update();
//                }

                // If we reached / exceeded a finite goal, clamp and stop
                if (_hits_to_mine != INDEFINITE && _number_hits >= static_cast<uint32_t>(_hits_to_mine))
                {
                    stopMiningInternal();
                }
            }
        }

        // Always update minerServo at end so PWM/system-level updates occur
        minerServo.update();
    }

    // Helper wrappers to set minerServo position
    /**
     * Chang the position of the miner to hit the button
     */
    void setMinerServoToPress()
    {
        minerServo.setAngle(MINER_SERVO_PRESS_ANGLE);
    }

    /**
     * Change the position of the miner to be fully in the robot
     */
    void setMinerServoToRetract()
    {
        minerServo.setAngle(MINER_SERVO_RETRACT_ANGLE);
    }


    /**
     * Change the posiiton of the miner to store the ramp
     */
    void setServosToStore()
    {
        minerServo.setAngle(MINER_SERVO_STORE_ANGLE);
        rampServo.setAngle(RAMP_SERVO_STORE_ANGLE);
    }

    /**
     * Change the posiiton of the ramp to idle
     */
    void setRampServoToLift()
    {
        rampServo.setAngle(RAMP_SERVO_LIFT_ANGLE);
    }

    /**
     * Change the posiiton of the ramp to passive
     */
    void setRampServoToPassive()
    {
        rampServo.setAngle(RAMP_SERVO_PASSIVE_ANGLE);
    }

    /**
     * Start the sequency of hitting/retracting
     */
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

    /**
     * Stop hitting the button
     */
    void stopMiningInternal()
    {
        _mode = OFF;
        _cycleStartTime = 0;
        _onStartTime = 0;
        _miningStartTime = 0;
        // leave _number_hits as-is so caller can read progress
        // ensure minerServo is retracted on next update (or we can retract immediately)
        setMinerServoToRetract();
        minerServo.update();
    }
};
