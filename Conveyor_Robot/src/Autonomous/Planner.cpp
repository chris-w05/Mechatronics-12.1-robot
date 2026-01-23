// Planner.cpp
#include "Planner.hpp"

Planner::Planner(Drive &drive)
    : _drive(drive),
      _driveStep(drive),
      _detectStep(),
      _rejectStep(),
      _mineStep(),
      _indexStep(),
      _placeStep(),
      _advanceStep()
{
}

AutoStep *Planner::planNext()
{
    Strategy &S = Strategy::getInstance();

    switch (_phase)
    {
    case DRIVE_TO_SCAN:
        //Needs to account for last position, then move the robot to the next dispenser
        // Example: drive to scan/mine position
        _driveStep.configure(/*meters*/ 0.25f, /*velocity*/ 0.35f);
        _phase = DETECT;
        return &_driveStep;

    case DETECT:
        // DetectBlockStep writes into _detected
        _detectStep.configure(/*ms*/ 150);
        _phase = DECIDE_AND_ACT;
        return &_detectStep;


    case DECIDE_AND_ACT:
    {
        //Robot is at mine, and about to intake the block

        //All of these steps need to be reconfigured. A single Step needs to be returned, so composite steps should be built for each step. 
        Strategy::IndexDirection decision = S.whereToIndexBlock(_detected);
        _slotDecision = decision;

        if (_slotDecision == Strategy::IndexDirection::REJECT)
        {
            // Old behavior: maybe keep sampling or dump. Keep existing behavior if you want.
            _phase = DETECT;     // try again or run existing reject behavior
            return &_rejectStep; // if you want to keep a ground dump step, otherwise repurpose as chest.
        }
        else if (_slotDecision == Strategy::IndexDirection::CHEST)
        {
            // CHEST path: mine it, then drive to chest and place
            _mineStep.configure(/*ms*/ 350); // mine first
            _phase = DRIVE_TO_CHEST;         // after mining, drive to chest
            return &_mineStep;
        }
        else
        {
            // ACCEPT -> mine it
            _mineStep.configure(/*ms*/ 350);
            _phase = INDEX;
            return &_mineStep;
        }
    }

    case INDEX:
        //Robot just finished mining and needs to figure out what to do with the block newly in possession
        _indexStep.configure(_slotDecision);
        _acceptedCount++;

        // Next: score when ready (or placeholder threshold)
        if (S.readyToScore() || _acceptedCount >= 3)
            _phase = DRIVE_TO_TABLE;
        else
            _phase = DETECT;

        return &_indexStep;

    case DRIVE_TO_TABLE:
        _driveStep.configure(0.50f, 0.35f);
        _phase = PLACE;
        return &_driveStep;

    case PLACE:
        _placeStep.configure(S.getNextItem());
        _phase = ADVANCE;
        return &_placeStep;

    case ADVANCE:
        _acceptedCount = 0;
        _phase = DRIVE_TO_SCAN;
        return &_advanceStep;

    case DONE:
    default:
        return nullptr;
    }
}
