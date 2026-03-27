/**
 * @file Planner.hpp
 * @brief High-level autonomous planner used with `ReplanStep`.
 *
 * `Planner` implements a simple phase-based state machine that coordinates
 * driving, detecting blocks, mining, indexing, and scoring.  It pre-allocates
 * all step objects as members (no heap allocation during the match) and
 * exposes a `planNext()` function compatible with `ReplanStep::PlanFn`.
 *
 * Phases progress in order:
 * DRIVE_TO_SCAN → DETECT → DECIDE_AND_ACT → INDEX
 * → DRIVE_TO_TABLE → PLACE → ADVANCE → DONE
 */
// Planner.hpp
#pragma once

#include "AutoStep.h"
#include <stdint.h>
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/Shooter.hpp"
#include "utils/Strategy.hpp"

#include "Steps/DriveDistance.hpp"
#include "Steps/DriveArc.hpp"
#include "Steps/CompositeStep.hpp"
#include "Steps/DriveDistance.hpp"
#include "Steps/DetectBlock.hpp"
#include "Steps/MineBlock.hpp"
#include "Steps/IndexBlock.hpp"
#include "Steps/RejectSequence.hpp"
#include "Steps/AdvanceCycleStep.hpp"
#include "DeferredStep.hpp" // Expected: DeferredStep(AutoStep* (*factory)(void*), void* ctx)
#include "Steps/PlaceOnTable.hpp"
#include "Steps/FollowLineStep.hpp"

/**
 * @brief Phase-based autonomous planner compatible with `ReplanStep`.
 *
 * Call `planThunk()` as the planning function for `ReplanStep` to drive the
 * full autonomous cycle until `planNext()` returns `nullptr`.
 */
class Planner
{
public:

    /**
     * @brief Construct the planner and bind it to the robot's subsystems.
     * @param drive    Reference to the drive subsystem.
     * @param miner    Reference to the miner subsystem.
     * @param shooter  Reference to the shooter subsystem.
     */
    explicit Planner(Drive &drive,
        Miner &miner,
        Shooter &shooter );

    /**
     * @brief Return the next step to execute, or `nullptr` when the routine is done.
     *
     * Called by `ReplanStep` after each child step finishes.  Advances the
     * internal phase and returns a pointer to the pre-allocated step object
     * that should run next.
     * @return Pointer to a pre-allocated `AutoStep`, or `nullptr` when DONE.
     */
    AutoStep *planNext();

    /**
     * @brief Thunk adaptor for use as a `ReplanStep::PlanFn`.
     * @param ctx  Pointer to a `Planner` instance (cast internally).
     * @return     Result of `planNext()` on the referenced Planner.
     */
    static AutoStep *planThunk(void *ctx)
    {
        return static_cast<Planner *>(ctx)->planNext();
    }

private:
    /** @brief Autonomous execution phase. */
    enum Phase : uint8_t
    {
        DRIVE_TO_SCAN,   ///< Drive to the block dispenser scan position
        DETECT,          ///< Detect which block type is present
        DECIDE_AND_ACT,  ///< Mine/interact with the block at the dispenser
        INDEX,           ///< Route the newly acquired block to the correct slot
        DRIVE_TO_TABLE,  ///< Drive to the scoring table
        DRIVE_TO_CHEST,  ///< Drive to the chest area (future use)
        PLACE,           ///< Place the block on the table
        ADVANCE,         ///< Advance the strategy cycle counter
        DONE             ///< All planned actions complete
    };


    Drive    &_drive;   ///< Reference to the drive subsystem
    Miner    &_miner;   ///< Reference to the miner subsystem
    Shooter  &_shooter; ///< Reference to the shooter subsystem

    Phase _phase = DRIVE_TO_SCAN; ///< Current autonomous execution phase

    Strategy::Block _detected = Strategy::Block::NONE;                      ///< Block type identified by the detector step
    Strategy::IndexDirection _slotDecision = Strategy::IndexDirection::REJECT; ///< Indexing decision for the current block

    uint8_t _acceptedCount = 0; ///< Number of accepted blocks since last score

    // Pre-allocated reusable steps (no heap allocation during the match)
    DriveDistance    _driveStep;    ///< Reusable straight-drive step
    DriveArc         _driveArcStep; ///< Reusable arc-drive step
    FollowLineStep   _followLineStep; ///< Reusable line-follow step
    MineBlock        _mineStep;     ///< Reusable mine-block step
    IndexBlockStep   _indexStep;    ///< Reusable block-indexing step
    PlaceOnTableStep _placeStep;    ///< Reusable place-on-table step
    AdvanceCycleStep _advanceStep;  ///< Reusable cycle-advance step
};
