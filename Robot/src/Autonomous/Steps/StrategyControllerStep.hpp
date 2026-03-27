/**
 * @file StrategyControllerStep.hpp
 * @brief High-level `AutoStep` that runs the full strategy loop inline.
 *
 * Rather than delegating to sub-steps, this step embeds the entire
 * DECIDE → GOTO_SCAN → SCAN → MINE → STORE → CRAFT state machine
 * inside a single `update()`.  It never finishes (`isFinished()` always
 * returns `false`) so it must be stopped externally.
 *
 * @note Stub implementations of `detectBlock()`, `mineBlock()`,
 * `indexBlock()`, `rejectBlock()`, and `runCraftSequence()` must be
 * replaced with real sensor/subsystem calls before competition use.
 */
// StrategyControllerStep.hpp
#ifndef STRATEGY_CONTROLLER_STEP_H
#define STRATEGY_CONTROLLER_STEP_H

#include "Autonomous/AutoStep.h"
#include "utils/Strategy.hpp"
#include "Robot.hpp"

/**
 * @brief Inline strategy state machine that decides, scans, mines, stores, and crafts.
 */
class StrategyControllerStep : public AutoStep
{
public:
    /**
     * @brief Construct the step.
     * @param robot  Reference to the top-level Robot object.
     */
    StrategyControllerStep(Robot &robot) : _robot(robot) {}

    void start() override
    {
        _state = State::DECIDE;
    }

    void update() override
    {
        using S = Strategy;
        Strategy &strat = S::getInstance();

        switch (_state)
        {
        case State::DECIDE:
            _currentCycle = strat.getNextItem();
            _state = State::GOTO_SCAN;
            break;

        case State::GOTO_SCAN:
            // drive to sensor position, or assume already there
            // If you have a step-based drive, you can push a CompositeStep here.
            _state = State::SCAN;
            break;

        case State::SCAN:
            if (!detectBlock(_lastDetected))
            {
                // nothing seen yet — keep scanning
                return;
            }
            // Decide where to put it
            _lastDecision = strat.whereToIndexBlock(_lastDetected);
            if (_lastDecision == Strategy::IndexDirection::REJECT)
            {
                // perform reject action (e.g. drop / ignore)
                rejectBlock();
                _state = State::DECIDE; // continue looking for useful blocks
            }
            else
            {
                _state = State::MINE;
            }
            break;

        case State::MINE:
            if (mineBlock(_lastDetected))
            {
                _state = State::STORE;
            }
            break;

        case State::STORE:
            if (indexBlock(_lastDecision))
            {
                // Tell strategy we now hold that block
                strat.setPossession(_lastDecision, _lastDetected);
                // If ready to craft, go craft
                if (strat.readyToScore())
                {
                    _state = State::CRAFT;
                }
                else
                {
                    _state = State::DECIDE;
                }
            }
            break;

        case State::CRAFT:
            runCraftSequence();
            strat.advanceCycle();
            _state = State::DECIDE;
            break;
        }
    }

    bool isFinished() const override
    {
        // Keep running until you explicitly decide to finish (e.g. pickaxe==DIAMOND and cycles done)
        return false;
    }

private:
    /** @brief Internal states of the strategy execution loop. */
    enum class State
    {
        DECIDE,    ///< Determine the next item to collect
        GOTO_SCAN, ///< Drive to the block scanner position
        SCAN,      ///< Poll the color sensor for a block
        MINE,      ///< Mine the detected block
        STORE,     ///< Index/store the mined block
        CRAFT      ///< Execute the crafting/scoring sequence
    };
    State _state = State::DECIDE; ///< Active state of the strategy loop

    Robot &_robot;                                              ///< Top-level Robot reference
    Strategy::CycleType _currentCycle;                          ///< Cycle item currently being pursued
    Strategy::Block _lastDetected = Strategy::Block::NONE;      ///< Most recently identified block type
    Strategy::IndexDirection _lastDecision = Strategy::IndexDirection::REJECT; ///< Indexing decision for last block

    /** @brief Poll the color sensor and populate `outBlock`. @return `true` when a block is positively identified. */
    bool detectBlock(Strategy::Block &outBlock)
    {
        // poll color sensor and map to Block enum
        // return true if detection happened (and set outBlock), false otherwise
        return false;
    }

    bool mineBlock(Strategy::Block target)
    {
        // call miner subsystem, wait for mining complete (use timers or step)
        return true;
    }

    bool indexBlock(Strategy::IndexDirection idx)
    {
        if (idx == Strategy::IndexDirection::REJECT)
            return false;
        // drive manipulator to index position, place block in bucket
        return true;
    }

    void rejectBlock()
    {
        // e.g., drive to disposal area, or simply ignore
    }

    void runCraftSequence()
    {
        // place blocks onto crafting table in the required index positions 0..8, call helper steps
    }
};

#endif
