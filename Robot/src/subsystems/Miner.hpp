/**
 * @file Miner.hpp
 * @brief Miner subsystem: servo-driven block-pressing mechanism and ramp deployment.
 *
 * The Miner subsystem controls two servos:
 * - **Miner servo** — presses down on dispenser blocks in a cyclic motion.
 * - **Ramp servo**  — deploys a ramp that funnels blocks toward the shooter.
 *
 * Mining runs in a timed cycle (MINER_CYCLE_MS) with a configurable press
 * duration (MINER_PRESS_MS).  Mining can be set to run for a fixed number of
 * press cycles or indefinitely with an optional safety timeout.
 */
#pragma once

#include <Arduino.h>
#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "utils/PID.hpp"
#include "Config.hpp"
#include "Devices/Solenoid.hpp"

/**
 * @brief Servo-driven block-mining and ramp-deployment subsystem.
 *
 * Operates as a state machine with four modes:
 * - **MINING** — Cyclic press-retract motion to knock blocks from the dispenser.
 * - **OFF**    — Servo retracted at rest position; no motion.
 * - **STORE**  — Servo and ramp both fully stowed for transport.
 * - **LIFT**   — Ramp elevated to receive incoming blocks.
 */
class Miner : public Subsystem
{
public:
    Miner(const int minerServoPin, const int rampServoPin)
        : minerServo(minerServoPin, 0, 180, false),
          rampServo(rampServoPin, 0, 180, false)
    {
    }

    /**
     * @brief Operating mode for the miner state machine.
     */
    enum Mode
    {
        LIFT,   ///< Ramp raised to funnel blocks; miner servo idle
        MINING, ///< Active press-retract cycling
        OFF,    ///< Miner servo retracted, ramp unset
        STORE   ///< Both servos stowed in transport position
    };

    /** @brief Sentinel value for `startMining()` meaning "run indefinitely". */
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
     * @brief Start mining for a fixed number of press cycles.
     * @param hits_to_mine  Number of press cycles to complete before stopping.
     *                      Negative values are treated as INDEFINITE.
     */
    void startMining(int32_t hits_to_mine)
    {
        if (hits_to_mine < 0)
            hits_to_mine = INDEFINITE;

        _hits_to_mine = hits_to_mine;
        startMining();
    }

    /** @brief Start mining indefinitely. Use `stopMining()` or hit-count exhaustion to stop. */
    // Start indefinite mining (preferred over magic numbers)
    void startMiningIndefinitely()
    {
        _hits_to_mine = INDEFINITE;
        startMining();
    }

    /** @brief Stop mining and retract miner servo. */
    void stopMining()
    {
        stopMiningInternal();
    }

    /** @brief Raise the ramp to its passive (deployed) angle. */
    void deployRamp(){
        setRampServoToPassive();
    }

    void stop() override
    {
        stopMiningInternal();
        setServosToStore();
    }

    /** @brief Return `true` when the miner has completed all requested hits (or timed out). */
    bool isDoneMining() const
    {
        return (_mode == OFF);
    }

    /**
     * @brief Stow both servos in the transport position and reset the hit counter.
     */
    void store()
    {
        _number_hits = 0; // Miner cannot move from retract unless _number_hits is 0
        _mode = STORE;
    }

    /** @return `true` if mining was stopped because the safety timeout elapsed. */
    bool hasTimedOut() const { return _timedOut; }

    /** @return Number of press cycles completed since `startMining()`. */
    uint32_t getHitsDone() const { return _number_hits; }

    /** @return Configured hit target (INDEFINITE = -1 for unlimited). */
    int32_t getHitsTarget() const { return _hits_to_mine; }

    /**
     * @brief Set the safety timeout for indefinite mining (0 to disable).
     * @param ms  Maximum continuous mining duration in milliseconds.  The default is 5 minutes.
     */
    void setMaxContinuousMillis(unsigned long ms) { _maxContinuousMillis = ms; }

    /** @brief Clear a previous timeout so mining can be restarted. */
    void clearTimeout()
    {
        _timedOut = false;
        _miningStartTime = 0;
    }

private:
    Mode _mode = OFF;            ///< Current operating mode
    ServoControl minerServo;     ///< Servo that presses down on the dispenser block
    ServoControl rampServo;      ///< Servo that deploys/stows the feed ramp
    unsigned long _now = 0;      ///< Snapshot of millis() taken at start of update()
    unsigned long _cycleStartTime = 0; ///< millis() at the start of the current press cycle
    unsigned long _onStartTime    = 0; ///< millis() when the current press window started
    uint32_t  _number_hits  = 0;      ///< Number of press cycles completed
    int32_t   _hits_to_mine = 10;     ///< Target hit count; INDEFINITE (-1) = unlimited

    unsigned long _maxContinuousMillis = 300000UL; ///< Safety timeout for indefinite mining (ms, default 5 min)
    unsigned long _miningStartTime     = 0;        ///< millis() when MINING mode was first entered
    bool _timedOut = false;                        ///< True if mining was halted by the safety timeout

    /** @brief Drive the miner servo to the press-down angle. */
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
    /** @brief Retract the miner servo to clear the dispenser for the next block. */
    void setMinerServoToRetract()
    {
        minerServo.setAngle(MINER_SERVO_RETRACT_ANGLE);
    }

    /** @brief Stow both the miner servo and the ramp servo in the transport position. */
    void setServosToStore()
    {
        minerServo.setAngle(MINER_SERVO_STORE_ANGLE);
        rampServo.setAngle(RAMP_SERVO_STORE_ANGLE);
    }

    /** @brief Raise the ramp to the block-catching (LIFT) angle. */
    void setRampServoToLift()
    {
        rampServo.setAngle(RAMP_SERVO_LIFT_ANGLE);
    }

    /** @brief Lower the ramp to the passive (neutral) angle between mining cycles. */
    void setRampServoToPassive()
    {
        rampServo.setAngle(RAMP_SERVO_PASSIVE_ANGLE);
    }

    /**
     * @brief Internal transition to MINING mode; resets all hit and cycle counters.
     * @note The actual mining timestamp is captured on the first call to update().
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
     * @brief Internal: transition to OFF mode, reset timers, and immediately retract the miner servo.
     * @note `_number_hits` is intentionally preserved after stopping so the caller can inspect progress.
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
