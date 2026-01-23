#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <vector>

#include "Config.hpp"
#include "subsystems/Subsystem.h"
#include "subsystems/Drive.hpp"
#include "Autonomous/AutonomousRoutine.hpp"
#include "Autonomous/Planner.hpp"
#include "subsystems/Arm.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/SerialComs.hpp"
#include "SoftwareSerial.h"
#include "Config.hpp"


class Robot
{
public:
    Robot();
    void init();
    void update();

private:
    Drive drive;
    Miner miner;
    SerialComs serialComs;

    Planner planner;
    AutonomousRoutine autonomous;
    Subsystem *subsystems[MAX_SUBSYSTEMS];
    int subsystemCount = 0;
};

#endif
