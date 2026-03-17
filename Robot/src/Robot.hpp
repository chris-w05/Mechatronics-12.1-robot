#include "Autonomous/Steps/DriveDistance.hpp"
#include "Autonomous/Steps/CompositeStep.hpp"
#include "Autonomous/Planner.hpp"
#include "Autonomous/ReplanStep.hpp"
#include "subsystems/Subsystem.h"
#include "subsystems/Drive.hpp"
#include "Autonomous/AutonomousRoutine.hpp"
#include "Autonomous/Planner.hpp"
#include "subsystems/Miner.hpp"
#include "subsystems/SerialComs.hpp"
#include "subsystems/Shooter.hpp"
#include "SoftwareSerial.h"
#include "Config.hpp"
#include "Devices/MotorController.hpp"
#include "Autonomous/Steps/FireStep.hpp"
#include "Autonomous/Steps/MineBlock.hpp"
#include "Autonomous/Steps/DelayStep.hpp"
#include "Autonomous/Steps/FollowLineStep.hpp"
#include "Autonomous/Steps/DriveRadiusAtVelocity.hpp"
#include "Autonomous/Steps/DriveLineUntilWall.hpp"
#include "Autonomous/Steps/AccelerateStraightLine.hpp"

class Robot
{
public:
    Robot()
        : drive(
              LEFT_ENCODER_A, LEFT_ENCODER_B,
              RIGHT_ENCODER_A, RIGHT_ENCODER_B,
              drivePins,
              DISTANCE_SENSOR_PIN, LINE_SENSOR_PINS),
          miner(MINER_SERVO_PIN),
          shooter(SHOOTER_LIMIT_PIN, SHOOTER_ENCODER_A, SHOOTER_ENCODER_B,
                  shooterPins),
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

        // clear USB serial input buffer state
        _usbBufLen = 0;
        _usbHasCommand = false;
        _usbBuffer[0] = '\0';

        for (int i = 0; i < subsystemCount; ++i)
        {
            if (subsystems[i])
            {
                subsystems[i]->init();
            }
        }

        Serial.println("Subsystems initialized");
    }

    void update()
    {
        // Read both command channels first
        serialComs.update();
        updateUsbSerial();

        // Process command from Serial2 / SerialComs
        if (serialComs.hasCommand())
        {
            const char *raw = serialComs.getCommand();
            processCommandString(raw, &Serial2);
        }

        // Process command from USB Serial
        if (_usbHasCommand)
        {
            _usbHasCommand = false;
            processCommandString(_usbBuffer, &Serial);
        }

        switch (mode)
        {
        case AWAIT:
            for (int i = 0; i < subsystemCount; ++i)
            {
                subsystems[i]->update();
            }
            break;

        case AUTONOMOUS:
            for (int i = 0; i < subsystemCount; ++i)
            {
                subsystems[i]->update();
            }
            autonomous.update();
            break;

        case SERIAL_TEST:
            for (int i = 0; i < subsystemCount; ++i)
            {
                subsystems[i]->update();
            }
            autonomous.update();
            break;
        }
    }

private:
    //----------------------------------- COMMAND INGEST -----------------------------------

    static const uint8_t USB_CMD_MAX_LEN = 48;
    char _usbBuffer[USB_CMD_MAX_LEN];
    uint8_t _usbBufLen = 0;
    bool _usbHasCommand = false;

    void updateUsbSerial()
    {
        while (Serial.available() > 0)
        {
            char c = (char)Serial.read();

            if (c == '\r')
                continue;

            if (c == '\n')
            {
                _usbBuffer[_usbBufLen] = '\0';
                _usbHasCommand = true;
                _usbBufLen = 0;
                return; // one command per update, same behavior as SerialComs
            }

            if (_usbBufLen < USB_CMD_MAX_LEN - 1)
            {
                _usbBuffer[_usbBufLen++] = c;
            }
            else
            {
                // overflow: discard partial command
                _usbBufLen = 0;
            }
        }
    }

    void reply(Stream *port, const char *msg)
    {
        if (port)
            port->println(msg);
        else
            Serial.println(msg);
    }

    void processCommandString(const char *raw, Stream *replyPort)
    {
        if (!raw || raw[0] == '\0')
            return;

        Serial.print("RCV: ");
        Serial.println(raw);

        char cmdChar = 0;
        float p1 = 0.0f, p2 = 0.0f;
        bool p1Valid = false, p2Valid = false;

        if (parseCmdWithUpToTwoFloats(raw, cmdChar, p1, p1Valid, p2, p2Valid))
        {
            handleGlobalCommand(cmdChar, replyPort);

            if (mode == SERIAL_TEST)
            {
                if (p1Valid && p2Valid)
                    handleSerialTestCommand(cmdChar, p1, p2, replyPort);
                else if (p1Valid)
                    handleSerialTestCommand(cmdChar, p1, true, replyPort);
                else
                    handleSerialTestCommand(cmdChar, replyPort);
            }
        }
        else
        {
            reply(replyPort, "Invalid command format");
        }
    }

    /**
     * Local parser so both Serial and SerialComs can use identical command syntax.
     */
    bool parseCmdWithUpToTwoFloats(const char *cmdStr,
                                   char &outCmd,
                                   float &outP1, bool &outP1Valid,
                                   float &outP2, bool &outP2Valid)
    {
        if (!cmdStr || cmdStr[0] == '\0')
            return false;

        char buf[USB_CMD_MAX_LEN];
        strncpy(buf, cmdStr, sizeof(buf));
        buf[sizeof(buf) - 1] = '\0';

        for (char *p = buf; *p; ++p)
        {
            if (*p == ',' || *p == ':' || *p == '\t')
                *p = ' ';
        }

        char *p = buf;
        while (*p && isspace((unsigned char)*p))
            ++p;
        if (!*p)
            return false;

        outCmd = *p++;
        outP1Valid = false;
        outP2Valid = false;
        outP1 = 0.0f;
        outP2 = 0.0f;

        auto parseFloatAt = [](char *&ptr, float &outVal) -> bool
        {
            while (*ptr && isspace((unsigned char)*ptr))
                ++ptr;
            if (!*ptr)
                return false;

            char *s = ptr;
            if (*s == '+' || *s == '-')
                ++s;
            if (!(isdigit((unsigned char)*s) || *s == '.'))
                return false;

            outVal = (float)atof(ptr);

            while (*ptr && !isspace((unsigned char)*ptr))
                ++ptr;
            return true;
        };

        float v;
        if (parseFloatAt(p, v))
        {
            outP1 = v;
            outP1Valid = true;

            if (parseFloatAt(p, v))
            {
                outP2 = v;
                outP2Valid = true;
            }
        }

        return true;
    }

    //----------------------------------- SERIAL STUFF -----------------------------------

    void handleGlobalCommand(char cmd, Stream *replyPort)
    {
        switch (cmd)
        {
        case 'S':
            if (mode != SERIAL_TEST)
            {
                mode = SERIAL_TEST;
                reply(replyPort, "Entered SERIAL_TEST mode. Send 'H' for help.");
            }
            break;

        case 'A':
            if (mode != AUTONOMOUS)
            {
                mode = AUTONOMOUS;
                float speed = 20;

                autonomous.add(new DriveDistance(drive, 8, 10));
                autonomous.add(new DriveDistance(drive, 4, 15));
                autonomous.add(new DriveDistance(drive, 4, speed));
                autonomous.add(new DriveRadiusAtVelocity(drive, speed, -18, 14.96649));
                autonomous.add(new DriveRadiusAtVelocity(drive, speed, 36, 26));
                autonomous.add(new DriveDistance(drive, 23.5, speed));
                autonomous.add(new DriveRadiusAtVelocity(drive, 10, -8, 11));
                autonomous.add(new DriveLineToWallStep(drive, 10, 200));
                autonomous.start();

                reply(replyPort, "Autonomous started.");
            }
            break;

        case '=':
            for (int i = 0; i < subsystemCount; ++i)
            {
                subsystems[i]->stop();
            }
            autonomous.stop();
            autonomous.reset();
            reply(replyPort, "Stopped all subsystems.");
            break;

        case 'a':
            mode = AUTONOMOUS;
            autonomous.reset();
            autonomous.start();
            reply(replyPort, "Autonomous reset and started.");
            break;

        default:
            if (mode != SERIAL_TEST)
            {
                char buf[64];
                snprintf(buf, sizeof(buf),
                         "Unknown/global cmd '%c' (S start serial, A start auton)", cmd);
                reply(replyPort, buf);
            }
            break;
        }
    }

    void handleSerialTestCommand(char cmd, Stream *replyPort)
    {
        switch (cmd)
        {
        case 'M':
            miner.startMiningIndefinitely();
            break;

        case 'm':
            autonomous.stop();
            miner.store();
            break;

        case 'L':
            drive.followRadiusAtVelocity(10, -18);
            break;

        case 'R':
            drive.followRadiusAtVelocity(10, 18);
            break;

            case 'Q':
                // drive.setSpeed(10.0);
                autonomous.clear();
                // autonomous.add(new DriveDistance(drive, 30.0f, 3.0f));
                // autonomous.add(new DriveArc(drive, 2 * PI, .5f, 0.0f, false));
                // autonomous.add(new DriveDistance(drive, -10.0f, -3.0f));
                // autonomous.start();
                drive.followLineHardset(200);
                Serial.println("Drive: Close loop control called.");
                break;
            case 'W':
                drive.followRadiusCCW( .5, 8);
                Serial.println("Drive: Close loop turning called.");
                break;
            case 'T':
                shooter.autoFire();
                break;
            case 'q':
                drive.setSpeed(0.0);
                Serial.println("Closed loop on 0 velocity called.");
                break;
            case 'l':
            case 'r':
            case 'd':
                autonomous.stop();
                drive.hardSetSpeed(0);
                Serial.println("Drive: stop() called.");
                break;
            case 'E':
                // Exit serial testing and go back to awaiting mode (stop subsystems if needed)
                autonomous.stop();
                mode = AWAIT;
                Serial.println("Exited SERIAL_TEST. Back to AWAIT.");
                break;

        case 'P':
            shooter.stopFiring();
            shooter.prime();
            break;

        case 'p':
            shooter.stopFiring();
            shooter.holdPosition(1.1);
            break;

        case 'F':
            shooter.fire();
            break;

        case 'f':
            autonomous.stop();
            break;

        case '1':
            Serial.print("Current distance sensor distance: ");
            Serial.println(drive.getDistanceSensorReading());
            break;

        case 'H':
            reply(replyPort, "SERIAL_TEST commands:");
            reply(replyPort, "  M : start miner");
            reply(replyPort, "  m : stop miner");
            reply(replyPort, "  D <speed> : drive at closed-loop speed");
            reply(replyPort, "  I <pwm> : hard-set drive output");
            reply(replyPort, "  O <speed> : follow line");
            reply(replyPort, "  C <dist> : approach wall distance");
            reply(replyPort, "  ! <distance> <velocity> : drive distance");
            reply(replyPort, "  @ <angle> <omega> : rotate");
            reply(replyPort, "  # <radius> <deg> : follow arc");
            reply(replyPort, "  $ <value> <index> : set PID constant");
            reply(replyPort, "  E : exit SERIAL_TEST -> AWAIT");
            break;

        default:
            break;
        }
    }

    void handleSerialTestCommand(char cmd, float param, bool paramValid, Stream *replyPort)
    {
        switch (cmd)
        {
        case 'D':
            drive.setSpeed(paramValid ? param : 10);
            break;

        case 'I':
            drive.hardSetSpeed(paramValid ? param : 120);
            break;

        case 'O':
            if (paramValid)
                drive.followLineHardset(param);
            else
                drive.followLine(4);
            break;

        case 'C':
            if (paramValid)
                drive.apporachDistance(param);
            else
                drive.apporachDistance(10.0);
            break;

        case 'l':
        case 'r':
        case 'd':
            autonomous.stop();
            drive.hardSetSpeed(0);
            break;

        default:
            handleSerialTestCommand(cmd, replyPort);
            break;
        }
    }

    void handleSerialTestCommand(char cmd, float p1, float p2, Stream *replyPort)
    {
        switch (cmd)
        {
        case '!':
        {
            autonomous.clear();
            autonomous.add(new DriveDistance(drive, p1, p2));
            autonomous.start();
            break;
        }

        case '@':
        {
            autonomous.clear();
            autonomous.add(new DriveArc(drive, p1, p2, 0));
            autonomous.start();
            break;
        }

        case '#':
        {
            p2 *= PI / 180.0f;
            float distance = (p1 + DRIVETRAIN_WIDTH / 2.0f) * p2;
            float velocity = distance / 3.0f;

            autonomous.clear();
            autonomous.add(new DriveRadiusAtVelocity(drive, velocity, p1 + DRIVETRAIN_WIDTH / 2.0f, p2));
            autonomous.start();
            break;
        }

        case '$':
        {
            // Expected format: $ <value> <index>
            float value = p1;
            short index = (short)p2;

            drive.setDriveMotorPIDConstant(value, index);

            char buf[64];
            snprintf(buf, sizeof(buf), "Set PID constant index %d to %.5f", index, (double)value);
            reply(replyPort, buf);
            break;
        }

        default:
            break;
        }
    }

    // ------------------------------------------------------------------------------------------------------------

    enum RobotMode
    {
        AUTONOMOUS,
        SERIAL_TEST,
        AWAIT
    };

    Drive drive;
    Miner miner;
    Shooter shooter;
    SerialComs serialComs;

    AutonomousRoutine autonomous;
    Subsystem *subsystems[MAX_SUBSYSTEMS];
    RobotMode mode = AWAIT;
    int subsystemCount = 0;
};