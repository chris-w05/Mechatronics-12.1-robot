// Planner.hpp
#pragma once

#include "AutoStep.h"
#include <stdint.h>
#include "subsystems/Drive.hpp"
#include "utils/Strategy.hpp"

#include "Steps/DriveDistance.hpp"
#include "Steps/CompositeStep.hpp"
#include "Steps/DriveDistance.hpp"
#include "Steps/DetectBlock.hpp"
#include "Steps/MineBlock.hpp"
#include "Steps/IndexBlock.hpp"
#include "Steps/RejectSequence.hpp"
#include "Steps/AdvanceCycleStep.hpp"
#include "DeferredStep.hpp" // Expected: DeferredStep(AutoStep* (*factory)(void*), void* ctx)
#include "Steps/PlaceOnTable.hpp"

class Planner
{
public:

    //Add all of the subsystems that would be used by the robot
    explicit Planner(Drive &drive);

    // Called repeatedly by ReplanStep.
    // Returns pointer to the next step to run, or nullptr when finished.
    AutoStep *planNext();

    // Adapter for ReplanStep signature: AutoStep* (*)(void*)
    static AutoStep *planThunk(void *ctx)
    {
        return static_cast<Planner *>(ctx)->planNext();
    }

private:
    enum Phase : uint8_t
    {
        DRIVE_TO_SCAN,
        DETECT,
        DECIDE_AND_ACT,
        INDEX,
        DRIVE_TO_TABLE,
        DRIVE_TO_CHEST,
        PLACE,
        ADVANCE,
        DONE
    };


    //Add more subsystems
    Drive &_drive;


    
    Phase _phase = DRIVE_TO_SCAN;

    Strategy::Block _detected = Strategy::Block::NONE;
    Strategy::IndexDirection _slotDecision = Strategy::IndexDirection::REJECT;

    // Placeholder progress counter (replace with Strategy possession tracking later)
    uint8_t _acceptedCount = 0;

    // Pre-allocated reusable steps (no new/delete during match)
    DriveDistance _driveStep;
    DetectBlockStep _detectStep;
    RejectBlockStep _rejectStep;
    MineBlockStep _mineStep;
    IndexBlockStep _indexStep;
    PlaceOnTableStep _placeStep;
    AdvanceCycleStep _advanceStep;
};
