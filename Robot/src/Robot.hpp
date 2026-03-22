// =============================================================================
// Robot.hpp
// Top-level robot class. Owns all subsystems and the autonomous scheduler.
//
// Serial command handling is implemented in Robot_Commands.hpp, which is
// included at the bottom of this file. See that file for the full command
// protocol reference.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "Config.hpp"

// Subsystems
#include "subsystems/Subsystem.h"
#include "subsystems/Drive.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/SerialComs.hpp"
#include "subsystems/Shooter.hpp"

// Autonomous framework and steps
#include "Autonomous/AutonomousRoutine.hpp"
#include "Autonomous/Planner.hpp"
#include "Autonomous/ReplanStep.hpp"
#include "Autonomous/Steps/AccelerateStraightLine.hpp"
#include "Autonomous/Steps/CompositeStep.hpp"
#include "Autonomous/Steps/DelayStep.hpp"
#include "Autonomous/Steps/DriveDistance.hpp"
#include "Autonomous/Steps/DriveLineUntilWall.hpp"
#include "Autonomous/Steps/DriveRadiusAngle.hpp"
#include "Autonomous/Steps/DriveRadiusAtVelocity.hpp"
#include "Autonomous/Steps/FireStep.hpp"
#include "Autonomous/Steps/FollowLineStep.hpp"
#include "Autonomous/Steps/MineBlock.hpp"
#include "Autonomous/Steps/ParallelStep.hpp"

// Devices / libraries
#include "Devices/MotorController.hpp"
#include "SoftwareSerial.h"

class Robot {
public:
    // =========================================================================
    // Subsystems (public for direct access from main / autonomous steps)
    // =========================================================================
    Drive      drive;
    Miner      miner;
    Shooter    shooter;
    SerialComs serialComs;

    // =========================================================================
    // Lifecycle
    // =========================================================================

    Robot()
        : drive(LEFT_ENCODER_A, LEFT_ENCODER_B,
                RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                drivePins, DISTANCE_SENSOR_PIN, LINE_SENSOR_PINS),
          miner(MINER_SERVO_PIN),
          shooter(SHOOTER_LIMIT_PIN, SHOOTER_ENCODER_A, SHOOTER_ENCODER_B, shooterPins),
          serialComs(Serial2)
    {
        subsystems[subsystemCount++] = &drive;
        subsystems[subsystemCount++] = &miner;
        subsystems[subsystemCount++] = &shooter;
        subsystems[subsystemCount++] = &serialComs;
    }

    void init()
    {
        Serial.println("Starting robot init");
        _usbBufLen     = 0;
        _usbHasCommand = false;
        _usbBuffer[0]  = '\0';

        for (int i = 0; i < subsystemCount; ++i)
            if (subsystems[i]) subsystems[i]->init();

        Serial.println("Subsystems initialized");
        printSerialHelp();
    }

    void update()
    {
        // Read both command channels
        serialComs.update();
        updateUsbSerial();

        if (serialComs.hasCommand())
            processCommandString(serialComs.getCommand(), &Serial2);

        if (_usbHasCommand) {
            _usbHasCommand = false;
            processCommandString(_usbBuffer, &Serial);
        }

        // Tick all subsystems every loop
        for (int i = 0; i < subsystemCount; ++i)
            subsystems[i]->update();

        // Tick the autonomous scheduler when active
        if (mode == AUTONOMOUS || mode == SERIAL_TEST)
            autonomous.update();
    }

private:
    // =========================================================================
    // Operating mode
    // =========================================================================
    enum RobotMode { AUTONOMOUS, SERIAL_TEST, AWAIT };

    // =========================================================================
    // State
    // =========================================================================
    AutonomousRoutine autonomous;
    Subsystem        *subsystems[MAX_SUBSYSTEMS];
    RobotMode         mode          = AWAIT;
    int               subsystemCount = 0;

    // =========================================================================
    // USB serial input buffer
    // =========================================================================
    static const uint8_t USB_CMD_MAX_LEN = 48;
    static const uint8_t CMD_TOKEN_MAX   = 16; // max chars in a command keyword
    char    _usbBuffer[USB_CMD_MAX_LEN];
    uint8_t _usbBufLen     = 0;
    bool    _usbHasCommand = false;

    // =========================================================================
    // Serial command handling — implemented in Robot_Commands.hpp
    // =========================================================================
    void updateUsbSerial();
    void reply(Stream *port, const char *msg);
    void processCommandString(const char *raw, Stream *replyPort);
    bool parseCmdWithUpToTwoFloats(const char *cmdStr,
                                   char  *outCmd,          // receives null-terminated token
                                   float &outP1, bool &outP1Valid,
                                   float &outP2, bool &outP2Valid);
    void printSerialHelp(Stream *port = nullptr);
    void handleGlobalCommand(const char *cmd, Stream *replyPort);
    void handleSerialTestCommand(const char *cmd, Stream *replyPort);
    void handleSerialTestCommand(const char *cmd, float param, bool paramValid, Stream *replyPort);
    void handleSerialTestCommand(const char *cmd, float p1, float p2, Stream *replyPort);
};

// Implementations of serial command methods — kept in a separate file
// so this header stays focused on the robot's structure.
#include "Robot_Commands.hpp"
