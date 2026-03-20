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

        if (c == '\r') continue;

        if (c == '\n') {
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

inline void Robot::reply(Stream *port, const char *msg)
{
    if (port) port->println(msg);
    else      Serial.println(msg);
}

// --- Command parsing ---------------------------------------------------------

/**
 * Parse a command string of the form:  <letter> [float] [float]
 * Delimiters between tokens may be spaces, commas, colons, or tabs.
 * Returns true if at least the command character was found.
 */
inline bool Robot::parseCmdWithUpToTwoFloats(const char *cmdStr,
                                              char  &outCmd,
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

    outCmd     = *p++;
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

    char  cmdChar = 0;
    float p1 = 0.0f, p2 = 0.0f;
    bool  p1Valid = false, p2Valid = false;

    if (!parseCmdWithUpToTwoFloats(raw, cmdChar, p1, p1Valid, p2, p2Valid)) {
        reply(replyPort, "Invalid command format");
        return;
    }

    handleGlobalCommand(cmdChar, replyPort);

    if (mode == SERIAL_TEST) {
        if      (p1Valid && p2Valid) handleSerialTestCommand(cmdChar, p1, p2,     replyPort);
        else if (p1Valid)            handleSerialTestCommand(cmdChar, p1, p1Valid, replyPort);
        else                         handleSerialTestCommand(cmdChar,              replyPort);
    }
}

// --- Global commands (active in all modes) -----------------------------------

inline void Robot::handleGlobalCommand(char cmd, Stream *replyPort)
{
    switch (cmd) {

    case 'S':
        if (mode != SERIAL_TEST) {
            mode = SERIAL_TEST;
            reply(replyPort, "Entered SERIAL_TEST mode. Send 'H' for help.");
        }
        break;

    case 'A':
        if (mode != AUTONOMOUS) {
            mode = AUTONOMOUS;
            float speed = 15;
            autonomous.add(new DriveDistance(   drive, 18, speed));
            autonomous.add(new DriveRadiusAngle(drive, speed*1, -18,  45));
            autonomous.add(new DriveRadiusAngle(drive, speed*1,  18, -45));
            autonomous.add(new DriveDistance(   drive, 26, speed));
            autonomous.add(new DriveRadiusAngle(drive, speed * .5, -15, 90));
            autonomous.add(new DriveDistance(   drive, 2, speed*.25));
            autonomous.add(new MineBlockStep(   miner, 10000000));
            autonomous.start();
            reply(replyPort, "Autonomous started.");
        }
        break;

    case '=':
        for (int i = 0; i < subsystemCount; ++i)
            subsystems[i]->stop();
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
        if (mode != SERIAL_TEST) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "Unknown cmd '%c'. Send 'S' for serial test, 'A' for autonomous.", cmd);
            reply(replyPort, buf);
        }
        break;
    }
}

// --- SERIAL_TEST commands (no parameters) ------------------------------------

inline void Robot::handleSerialTestCommand(char cmd, Stream *replyPort)
{
    switch (cmd) {
    case 'M': miner.startMiningIndefinitely();                break;
    case 'm': autonomous.stop(); miner.store();               break;
    case 'L': drive.followRadiusAtVelocity(10, -18);          break;
    case 'R': drive.followRadiusAtVelocity(10,  18);          break;
    case 'Q':
        autonomous.clear();
        drive.followLineHardset(200);
        break;
    case 'W': drive.followRadiusCCW(0.5f, 8);                 break;
    case 'T': shooter.autoFire();                              break;
    case 'q': drive.setSpeed(0.0f);                            break;
    case 'l':
    case 'r':
    case 'd':
        autonomous.stop();
        drive.hardSetSpeed(0);
        break;
    case 'E':
        autonomous.stop();
        mode = AWAIT;
        Serial.println("Exited SERIAL_TEST. Back to AWAIT.");
        break;
    case 'P': shooter.stopFiring(); shooter.prime();           break;
    case 'p': shooter.stopFiring(); shooter.holdPosition(1.1); break;
    case 'F': shooter.fire();                                  break;
    case 'f': autonomous.stop();                               break;
    case '1':
        Serial.print("Distance sensor: ");
        Serial.println(drive.getDistanceSensorReading());
        break;
    case 'H':
        reply(replyPort, "SERIAL_TEST commands:");
        reply(replyPort, "  M               : start miner");
        reply(replyPort, "  m               : stop miner");
        reply(replyPort, "  D <speed>        : closed-loop drive speed (in/s)");
        reply(replyPort, "  I <pwm>          : hard-set drive output (raw signal)");
        reply(replyPort, "  O <speed>        : follow line");
        reply(replyPort, "  C <dist>         : approach wall at distance (cm)");
        reply(replyPort, "  ! <dist> <vel>   : drive distance step");
        reply(replyPort, "  @ <angle> <omega>: rotate step");
        reply(replyPort, "  # <radius> <deg> : follow arc step");
        reply(replyPort, "  $ <val> <index>  : set PID constant");
        reply(replyPort, "  E                : exit SERIAL_TEST -> AWAIT");
        break;
    default:
        break;
    }
}

// --- SERIAL_TEST commands (one float parameter) ------------------------------

inline void Robot::handleSerialTestCommand(char cmd, float param, bool paramValid,
                                            Stream *replyPort)
{
    switch (cmd) {
    case 'D': drive.setSpeed(paramValid ? param : 10.0f);         break;
    case 'I': drive.hardSetSpeed(paramValid ? (int16_t)param : 120); break;
    case 'O':
        if (paramValid) drive.followLineHardset((int)param);
        else            drive.followLine(4.0f);
        break;
    case 'C': drive.approachDistance(paramValid ? param : 10.0f); break;
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

// --- SERIAL_TEST commands (two float parameters) -----------------------------

inline void Robot::handleSerialTestCommand(char cmd, float p1, float p2,
                                            Stream *replyPort)
{
    switch (cmd) {

    case '!':
        autonomous.clear();
        autonomous.add(new DriveDistance(drive, p1, p2));
        autonomous.start();
        break;

    case '@':
        autonomous.clear();
        autonomous.add(new DriveArc(drive, p1, p2, 0));
        autonomous.start();
        break;

    case '#': {
        // p1 = radius (inches), p2 = arc angle (degrees)
        float angleRad = p2 * PI / 180.0f;
        float halfW    = DRIVETRAIN_WIDTH / 2.0f;
        float arcRadius = (p1 > 0) ? p1 + halfW : p1 - halfW;
        float distance  = arcRadius * angleRad;
        float velocity  = distance / 4.0f;
        autonomous.clear();
        autonomous.add(new DriveRadiusAtVelocity(drive, velocity, arcRadius, distance));
        autonomous.start();
        break;
    }

    case '$': {
        drive.setDriveMotorPIDConstant(p1, (int)p2);
        char buf[64];
        snprintf(buf, sizeof(buf), "Set PID constant index %d to %.5f", (int)p2, (double)p1);
        reply(replyPort, buf);
        break;
    }

    default:
        break;
    }
}
