// Planner.cpp
#include "Planner.hpp"

Planner::Planner(Drive &drive, Miner &miner, Shooter &shooter)
    : _drive(drive),
      _miner(miner),
      _shooter(shooter),
      _driveStep(drive),
      _driveArcStep(drive),
      _followLineStep(drive),
      _mineStep(miner, 5),
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
        _mineStep.configure(5);
        _phase = DECIDE_AND_ACT;
        return &_mineStep;


    case DECIDE_AND_ACT:
    {
        //Robot is at mine, and about to intake the block
        // ACCEPT -> mine it
        _mineStep.configure(/*ms*/ 350);
        _phase = INDEX;
        return &_mineStep;
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
