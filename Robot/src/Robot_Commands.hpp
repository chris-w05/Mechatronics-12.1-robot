/**
 * @file Robot_Commands.hpp
 * @brief Serial command protocol implementation for the `Robot` class.
 *
 * This file is `#include`d at the bottom of Robot.hpp — it is **not** a
 * standalone header.  It provides `inline` implementations of all the
 * private serial-command methods declared in `Robot`.
 *
 * ### Command Protocol Summary
 * Commands are newline-terminated ASCII strings.  Two ports are monitored:
 *   - **USB Serial** (`Serial`)   — direct PC connection.
 *   - **Serial2**                 — Arduino-to-Arduino via `SerialComs`.
 *
 * | Command            | Mode        | Action |
 * |--------------------|-------------|--------|
 * | `A`                | Any         | Start autonomous routine |
 * | `a`                | Any         | Reset & restart autonomous |
 * | `S`                | Any         | Enter SERIAL_TEST mode |
 * | `stop`             | Any         | Stop all subsystems |
 * | `Help`             | Any         | Print this command reference |
 * | `E`                | SERIAL_TEST | Exit to AWAIT mode |
 * | `Mine`             | SERIAL_TEST | Start miner indefinitely |
 * | `m`                | SERIAL_TEST | Stop miner (store) |
 * | `Drive <sp>`       | SERIAL_TEST | Closed-loop drive at sp in/s |
 * | `Linefollow [pwm]` | SERIAL_TEST | Start line-following |
 * | `Distance <d>`     | SERIAL_TEST | Approach wall at d inches |
 * | `Approach <d>`     | SERIAL_TEST | Approach along line to d inches |
 * | `Line <d> <sp>`    | SERIAL_TEST | Queue DriveDistance step |
 * | `Turn <ang> <sp>`  | SERIAL_TEST | Queue in-place arc turn |
 * | `Arc <r> <deg>`    | SERIAL_TEST | Queue radius-arc step |
 * | `SetPID <v> <i>`   | SERIAL_TEST | Set drive PID constant |
 * | `Fire`             | SERIAL_TEST | Start shooter autofire |
 * | `F`                | SERIAL_TEST | Fire one shot |
 * | `f`                | SERIAL_TEST | Stop firing, prime shooter |
 * | `p`                | SERIAL_TEST | Hold shooter position |
 * | `ReadDistance`     | SERIAL_TEST | Print distance sensor reading |
 * | `ramsete`          | SERIAL_TEST | Toggle Ramsete heading correction |
 */
// Robot_Commands.hpp
// =============================================================================
// Serial command protocol implementation for the Robot class.
//
// This file is included at the bottom of Robot.hpp — it is NOT a standalone
// header. It implements the private command-handling methods declared in Robot.
// =============================================================================

// --- USB serial ingestion ----------------------------------------------------

inline void Robot::updateUsbSerial()
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        // Treat CR as command terminator too (handles CR-only line-ending settings).
        if (c == '\r') c = '\n';

        if (c == '\n') {
            if (_usbBufLen == 0) continue;
            _usbBuffer[_usbBufLen] = '\0';
            _usbHasCommand = true;
            _usbBufLen = 0;
            return; // process one command per update() call
        }

        if (_usbBufLen < USB_CMD_MAX_LEN - 1)
            _usbBuffer[_usbBufLen++] = c;
        else
            _usbBufLen = 0; // overflow: discard partial command
    }
}

inline void Robot::printSerialHelp(Stream *port)
{
    reply(port, "  Robot Serial Command Reference");
    reply(port, "--- Global (all modes) ---------------");
    reply(port, "  Help                 : print this help");
    reply(port, "  S                    : enter SERIAL_TEST mode");
    reply(port, "  A                    : start autonomous routine");
    reply(port, "  a                    : reset & restart autonomous");
    reply(port, "  stop                 : stop all subsystems");
    reply(port, "------- SERIAL_TEST mode -------------");
    reply(port, "     E                 : exit to AWAIT mode");
    reply(port, "     Mine              : start miner (runs indefinitely)");
    reply(port, "     m                 : stop miner");
    reply(port, "     ramsete           : toggle Ramsete heading correction");
    reply(port, "     ReadDistance      : print distance sensor reading");
    reply(port, "     Fire              : auto-fire shooter");
    reply(port, "     F                 : fire shooter (manual)");
    reply(port, "     f                 : stop firing, prime shooter");
    reply(port, "     p                 : stop firing, hold position");
    reply(port, "     Drive <sp>        : closed-loop drive (in/s)");
    reply(port, "     Linefollow <d>    : follow line (optional PWM override)");
    reply(port, "     Distance <d>      : approach wall at distance (in)");
    reply(port, "     Approach <d>      : approach along line");
    reply(port, "     Line <d> <sp>     : queue drive-distance step (in, in/s)");
    reply(port, "     Turn <ang> <sp>   : queue in-place turn (rad, rad/s)");
    reply(port, "     Arc <r> <deg>     : queue arc turn (radius in, angle deg)");
    reply(port, "     SetPID <val> <i>  : set drive PID constant [value, index]");
    reply(port, "======================================");
}

inline void Robot::reply(Stream *port, const char *msg)
{
    if (port) port->println(msg);
    else      Serial.println(msg);
}

// --- Command parsing ---------------------------------------------------------

/**
 * @brief Parse a command string of the form `<keyword> [float] [float]`.
 *
 * Keywords may be multi-character (up to CMD_TOKEN_MAX-1 chars).
 * Delimiters between tokens may be spaces, commas, colons, or tabs.
 * @return `true` if at least the command keyword was found.
 */
inline bool Robot::parseCmdWithUpToTwoFloats(const char *cmdStr,
                                              char  *outCmd,
                                              float &outP1, bool &outP1Valid,
                                              float &outP2, bool &outP2Valid)
{
    if (!cmdStr || cmdStr[0] == '\0') return false;

    char buf[USB_CMD_MAX_LEN];
    strncpy(buf, cmdStr, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';

    // Normalize delimiters to spaces
    for (char *p = buf; *p; ++p)
        if (*p == ',' || *p == ':' || *p == '\t') *p = ' ';

    char *p = buf;
    while (*p && isspace((unsigned char)*p)) ++p;
    if (!*p) return false;

    // Read the full command keyword (all non-whitespace chars)
    int tokLen = 0;
    while (*p && !isspace((unsigned char)*p) && tokLen < CMD_TOKEN_MAX - 1)
        outCmd[tokLen++] = *p++;
    while (*p && !isspace((unsigned char)*p)) ++p; // skip overflow chars
    outCmd[tokLen] = '\0';
    if (tokLen == 0) return false;

    outP1Valid = outP2Valid = false;
    outP1 = outP2 = 0.0f;

    auto parseFloat = [](char *&ptr, float &val) -> bool {
        while (*ptr && isspace((unsigned char)*ptr)) ++ptr;
        if (!*ptr) return false;
        char *s = ptr;
        if (*s == '+' || *s == '-') ++s;
        if (!(isdigit((unsigned char)*s) || *s == '.')) return false;
        val = (float)atof(ptr);
        while (*ptr && !isspace((unsigned char)*ptr)) ++ptr;
        return true;
    };

    if (parseFloat(p, outP1)) {
        outP1Valid = true;
        if (parseFloat(p, outP2))
            outP2Valid = true;
    }
    return true;
}

inline void Robot::processCommandString(const char *raw, Stream *replyPort)
{
    if (!raw || raw[0] == '\0') return;

    Serial.print("RCV: "); Serial.println(raw);

    char  cmdStr[CMD_TOKEN_MAX] = "";
    float p1 = 0.0f, p2 = 0.0f;
    bool  p1Valid = false, p2Valid = false;

    if (!parseCmdWithUpToTwoFloats(raw, cmdStr, p1, p1Valid, p2, p2Valid)) {
        reply(replyPort, "Invalid command format");
        return;
    }

    handleGlobalCommand(cmdStr, replyPort);

    if (mode == SERIAL_TEST) {
        if      (p1Valid && p2Valid) handleSerialTestCommand(cmdStr, p1, p2,     replyPort);
        else if (p1Valid)            handleSerialTestCommand(cmdStr, p1, p1Valid, replyPort);
        else                         handleSerialTestCommand(cmdStr,              replyPort);
    }
}

// --- Global commands (active in all modes) -----------------------------------

inline void Robot::handleGlobalCommand(const char *cmd, Stream *replyPort)
{
    if (strcmp(cmd, "A") == 0)
    {
        if (mode != AUTONOMOUS)
        {
            mode = AUTONOMOUS;
            float speed = 15.5;
            autonomous.add(new DriveDistance(drive, 14, speed));
            autonomous.add(new DriveRadiusAngle(drive, speed * 1, -20, 45));
            autonomous.add(new DriveDistance(drive, 8, speed));
            autonomous.add(new DriveRadiusAngle(drive, speed * 1, 20, -45));
            autonomous.add(new DriveDistance(drive, 31.75, speed));
            autonomous.add(new DriveRadiusToLine(drive, speed * .2, -3));
            // autonomous.add(new DriveDistance(drive, 2, speed * .3));
            autonomous.add(new DriveLineToWallStep(drive, 1));
            autonomous.add(new MineBlockAutofire(miner, shooter, 5 * 60 ));

            autonomous.start();
            reply(replyPort, "Autonomous started.");
        }
    }
    else if (strcmp(cmd, "S") == 0) {
        if (mode != SERIAL_TEST) {
            autonomous.stop();
            mode = SERIAL_TEST;
            reply(replyPort, "Entered SERIAL_TEST mode. Send 'H' for help.");
        }
    }
    
    else if (strcmp(cmd, "stop") == 0) {
        for (int i = 0; i < subsystemCount; ++i)
            subsystems[i]->stop();
        autonomous.stop();
        reply(replyPort, "Stopped all subsystems.");
    }
    else if (strcmp(cmd, "a") == 0) {
        mode = AUTONOMOUS;
        autonomous.reset();
        autonomous.start();
        reply(replyPort, "Autonomous reset and started.");
    }
    else if (strcmp(cmd, "Help") == 0)
    {
        printSerialHelp(replyPort);
    }
    else {
        if (mode != SERIAL_TEST) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "Unknown cmd '%s'. Send 'S' for serial test, 'A' for autonomous.", cmd);
            reply(replyPort, buf);
        }
    }
}

// --- SERIAL_TEST commands (no parameters) ------------------------------------

inline void Robot::handleSerialTestCommand(const char *cmd, Stream *replyPort)
{
    if      (strcmp(cmd, "Mine") == 0) {
        autonomous.stop();
        miner.startMiningIndefinitely();
        reply(replyPort, "Mine: miner started (indefinite).");
    }
    else if (strcmp(cmd, "m") == 0) {
        autonomous.stop();
        miner.store();
        reply(replyPort, "Mine: miner stopped (store mode).");
    }
    else if (strcmp(cmd, "ramsete") == 0) { drive.toggleRamseteCorrection(); }
    else if (strcmp(cmd, "Linefollow") == 0) {
        autonomous.clear();
        drive.followLineHardset(200);
    }
    else if (strcmp(cmd, "Fire") == 0) { shooter.autoFire(); }
    else if (strcmp(cmd, "l") == 0 || strcmp(cmd, "r") == 0 || strcmp(cmd, "d") == 0) {
        autonomous.stop();
        drive.hardSetSpeed(0);
    }
    else if (strcmp(cmd, "E") == 0) {
        autonomous.stop();
        mode = AWAIT;
        Serial.println("Exited SERIAL_TEST. Back to AWAIT.");
    }
    else if (strcmp(cmd, "f") == 0) { shooter.stopFiring(); shooter.prime(); }
    else if (strcmp(cmd, "p") == 0) { shooter.stopFiring(); shooter.holdPosition(1.1); }
    else if (strcmp(cmd, "F") == 0) { shooter.fire(); }
    else if (strcmp(cmd, "ReadDistance") == 0) {
        Serial.print("Distance sensor: ");
        Serial.println(drive.getDistanceSensorReading());
    }
}

// --- SERIAL_TEST commands (one float parameter) ------------------------------

inline void Robot::handleSerialTestCommand(const char *cmd, float param, bool paramValid,
                                            Stream *replyPort)
{
    if      (strcmp(cmd, "Drive") == 0) { drive.setSpeed(paramValid ? param : 10.0f); }
    else if (strcmp(cmd, "Linefollow") == 0) {
        if (paramValid) drive.followLineHardset(paramValid ? (int)param : 100);
    }
    else if (strcmp(cmd, "Distance") == 0) { drive.approachDistance(paramValid ? param : 10.0f); }
    else if (strcmp(cmd, "Approach") == 0) { drive.approachAlongLine(paramValid ? param : 2.0f); }
    else if (strcmp(cmd, "l") == 0 || strcmp(cmd, "r") == 0 || strcmp(cmd, "d") == 0)
    {
        autonomous.stop();
        drive.hardSetSpeed(0);
    }
    else {
        handleSerialTestCommand(cmd, replyPort);
    }
}

// --- SERIAL_TEST commands (two float parameters) -----------------------------

inline void Robot::handleSerialTestCommand(const char *cmd, float p1, float p2,
                                            Stream *replyPort)
{
    if (strcmp(cmd, "Line") == 0) {
        autonomous.clear();
        autonomous.add(new DriveDistance(drive, p1, p2));
        autonomous.start();
    }
    else if (strcmp(cmd, "Turn") == 0) {
        autonomous.clear();
        autonomous.add(new DriveArc(drive, p1, p2, 0));
        autonomous.start();
    }
    else if (strcmp(cmd, "Arc") == 0) {
        // p1 = radius (inches), p2 = arc angle (degrees)
        float angleRad = p2 * PI / 180.0f;
        float halfW    = DRIVETRAIN_WIDTH / 2.0f;
        float arcRadius = (p1 > 0) ? p1 + halfW : p1 - halfW;
        float distance  = arcRadius * angleRad;
        float velocity  = distance / 4.0f;
        autonomous.clear();
        autonomous.add(new DriveRadiusAtVelocity(drive, velocity, arcRadius, distance));
        autonomous.start();
    }
    else if (strcmp(cmd, "SetPID") == 0) {
        drive.setDriveMotorPIDConstant(p1, (int)p2);
        char buf[64];
        snprintf(buf, sizeof(buf), "Set PID constant index %d to %.5f", (int)p2, (double)p1);
        reply(replyPort, buf);
    }
}
