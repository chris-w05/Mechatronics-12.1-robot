/**
 * @file Robot.hpp
 * @brief Top-level robot class that owns all subsystems and the autonomous scheduler.
 *
 * `Robot` is the single entry-point for both `setup()` and `loop()`.
 * It owns `Drive`, `Miner`, `Shooter`, and `SerialComs` as public members
 * so that autonomous steps can reference them directly.
 *
 * Serial command handling (USB + Serial2) is implemented in Robot_Commands.hpp,
 * which is `#include`d at the bottom of this file.  See that file for the full
 * command protocol reference.
 */
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
#include "Autonomous/Steps/DeployRamp.hpp"
#include "Autonomous/Steps/MineBlockAutofire.hpp"

// Devices / libraries
#include "Devices/MotorController.hpp"
#include "SoftwareSerial.h"

/**
 * @brief Aggregates all robot subsystems and drives the main control loop.
 */
class Robot {
public:
    Drive      drive;      ///< Differential drivetrain with odometry and line-follow
    Miner      miner;      ///< Block-dispenser mining mechanism (two servos)
    Shooter    shooter;    ///< Rack-and-pinion block shooter
    SerialComs serialComs; ///< Arduino-to-Arduino serial command receiver (Serial2)

    // =========================================================================
    // Lifecycle
    // =========================================================================
    /** @brief Construct the Robot, wiring subsystems to their hardware pins from Config.hpp. */
    Robot()
        : drive(LEFT_ENCODER_A, LEFT_ENCODER_B,
                RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                drivePins, DISTANCE_SENSOR_PIN, LINE_SENSOR_PINS),
          miner(MINER_SERVO_PIN, RAMP_SERVO_PIN),
          shooter(SHOOTER_LIMIT_PIN, SHOOTER_ENCODER_A, SHOOTER_ENCODER_B, shooterPins),
          serialComs(Serial2)
    {
        subsystems[subsystemCount++] = &drive;
        subsystems[subsystemCount++] = &miner;
        subsystems[subsystemCount++] = &shooter;
        subsystems[subsystemCount++] = &serialComs;
    }

    /**
     * @brief Initialise all subsystems — call once in `setup()`.
     *
     * Resets the USB serial buffer, calls `init()` on each subsystem, and
     * prints the serial command reference over USB.
     */
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

    /**
     * @brief Tick the robot — call every `loop()` iteration.
     *
     * Polls both serial ports, dispatches commands, updates all subsystems,
     * and advances the autonomous scheduler when active.
     */
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
    /** @brief High-level operating state for the robot. */
    enum RobotMode {
        AUTONOMOUS,  ///< Running the autonomous routine
        SERIAL_TEST, ///< Accepting live serial commands for subsystem testing
        AWAIT        ///< Idle — waiting for a mode-change command
    };

    // =========================================================================
    // State
    // =========================================================================
    AutonomousRoutine autonomous;              ///< Sequential autonomous step runner
    Subsystem        *subsystems[MAX_SUBSYSTEMS]; ///< Flat array of all subsystem pointers
    RobotMode         mode          = AWAIT;   ///< Current operating mode
    int               subsystemCount = 0;      ///< Number of subsystems registered

    // =========================================================================
    // USB serial input buffer
    // =========================================================================
    static const uint8_t USB_CMD_MAX_LEN = 48; ///< Max bytes in a USB serial command (including NUL)
    static const uint8_t CMD_TOKEN_MAX   = 16; ///< Max chars in a command keyword token
    char    _usbBuffer[USB_CMD_MAX_LEN];        ///< Accumulation buffer for the USB serial command
    uint8_t _usbBufLen     = 0;                 ///< Bytes currently in _usbBuffer
    bool    _usbHasCommand = false;             ///< True when a complete USB command is ready

    // =========================================================================
    // Serial command handling — implemented in Robot_Commands.hpp
    // =========================================================================
    /** @brief Non-blocking USB serial reader — appends to `_usbBuffer` until newline. */
    void updateUsbSerial();
    /**
     * @brief Send a message to a serial port (defaults to USB Serial if `port` is null).
     * @param port  Destination stream, or `nullptr` for USB Serial.
     * @param msg   Null-terminated string to transmit.
     */
    void reply(Stream *port, const char *msg);
    /**
     * @brief Parse and dispatch a raw command string to the appropriate handler.
     * @param raw        Null-terminated command string.
     * @param replyPort  Stream to echo responses back on.
     */
    void processCommandString(const char *raw, Stream *replyPort);
    /**
     * @brief Parse a keyword + up to two float parameters from a command string.
     * @param cmdStr      Input string.
     * @param outCmd      Output buffer for the keyword (size >= CMD_TOKEN_MAX).
     * @param outP1       First numeric parameter output.
     * @param outP1Valid  True if the first parameter was parsed.
     * @param outP2       Second numeric parameter output.
     * @param outP2Valid  True if the second parameter was parsed.
     * @return `true` if at least a keyword was extracted.
     */
    bool parseCmdWithUpToTwoFloats(const char *cmdStr,
                                   char  *outCmd,
                                   float &outP1, bool &outP1Valid,
                                   float &outP2, bool &outP2Valid);
    /** @brief Print the serial command reference to `port` (defaults to USB Serial). */
    void printSerialHelp(Stream *port = nullptr);
    /** @brief Handle commands that are valid in any operating mode. */
    void handleGlobalCommand(const char *cmd, Stream *replyPort);
    /** @brief Handle SERIAL_TEST commands with no numeric parameters. */
    void handleSerialTestCommand(const char *cmd, Stream *replyPort);
    /** @brief Handle SERIAL_TEST commands with one numeric parameter. */
    void handleSerialTestCommand(const char *cmd, float param, bool paramValid, Stream *replyPort);
    /** @brief Handle SERIAL_TEST commands with two numeric parameters. */
    void handleSerialTestCommand(const char *cmd, float p1, float p2, Stream *replyPort);
};

// Implementations of serial command methods — kept in a separate file
// so this header stays focused on the robot's structure.
#include "Robot_Commands.hpp"
