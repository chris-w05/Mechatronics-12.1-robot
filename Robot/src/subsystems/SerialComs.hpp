#ifndef SERIAL_COMS_HPP
#define SERIAL_COMS_HPP

#include <Arduino.h>

/**
 * SerialComs
 *
 * Simple command receiver for Arduino-to-Arduino communication.
 * Commands are newline-terminated ASCII strings.
 *
 * Usage:
 *   SerialComs serialComs(Serial1);
 *   serialComs.init();
 *   // in loop:
 *   serialComs.update();
 *   if (serialComs.hasCommand()) {
 *     const char *cmd = serialComs.getCommand();
 *     // or char c = serialComs.getCommandChar();
 *   }
 */
class SerialComs : public Subsystem
{
public:
    static const uint8_t MAX_CMD_LEN = 32;

    /**
     * @param serial Reference to a HardwareSerial (Serial, Serial1, etc)
     */
    SerialComs(HardwareSerial &serialPort)
        : _serial(serialPort), _bufLen(0), _hasCommand(false)
    {
        _buffer[0] = '\0';
    }

    /**
     * Call once in setup()
     */
    void init() override
    {
        Serial.println("Serial communications initialized");
        // Nothing else required here; the HardwareSerial should be begun by the caller.
    }

    /**
     * Call periodically in loop()
     * Non-blocking. Buffers up to MAX_CMD_LEN-1 chars and sets a flag when newline ('\n') is seen.
     * Processes at most one complete command per call (returns after setting _hasCommand).
     */
    void update() override
    {
        // Serial.println("Updating coms");
        while (_serial.available() > 0)
        {
            char c = (char)_serial.read();

            // Ignore CR
            if (c == '\r')
                continue;

            // End of command
            if (c == '\n')
            {
                // Null-terminate and mark command available
                _buffer[_bufLen] = '\0';
                _hasCommand = true;
                _bufLen = 0; // prepare for next command
                return;      // deliver at most one command per update()
            }

            // Accumulate character if space remains
            if (_bufLen < MAX_CMD_LEN - 1)
            {
                _buffer[_bufLen++] = c;
            }
            else
            {
                // Buffer overflow -> discard current buffer to avoid malformed commands
                _bufLen = 0;
            }
        }
    }

    /**
     * @return true if a new command is available
     */
    bool hasCommand() const
    {
        return _hasCommand;
    }

    /**
     * Get the latest command string (pointer).
     * Calling this clears the command flag.
     */
    const char *getCommand()
    {
        _hasCommand = false;
        return _buffer;
    }

    /**
     * Convenience: return the first char of the latest command (or 0 if none).
     * Clears the command flag.
     */
    char getCommandChar()
    {
        if (!_hasCommand)
            return 0;
        _hasCommand = false;
        if (_buffer[0] == '\0')
            return 0;
        return _buffer[0];
    }

    /**
     * Send a message back (adds newline).
     */
    void send(const char *msg)
    {
        _serial.println(msg);
    }

    virtual void stop() override {}

    /**
     * Parse a single-char command optionally followed by a numeric parameter.
     * Accepts formats like:
     *   D 10
     *   D:10
     *   D,10
     *   D10
     *
     * Returns true if a command was parsed. If a numeric parameter exists, outParamIsValid is true and outParam contains it.
     */
    bool parseCmdWithOptionalFloat(const char *cmdStr, char &outCmd, float &outParam, bool &outParamIsValid)
    {
        if (!cmdStr || cmdStr[0] == '\0')
            return false;

        // Copy into modifiable buffer and normalize separators to spaces
        char buf[SerialComs::MAX_CMD_LEN];
        strncpy(buf, cmdStr, sizeof(buf));
        buf[sizeof(buf) - 1] = '\0';
        for (char *p = buf; *p; ++p)
            if (*p == ',' || *p == ':')
                *p = ' ';

        // Skip leading whitespace
        char *p = buf;
        while (*p && isspace((unsigned char)*p))
            ++p;
        if (!*p)
            return false;

        // First non-space char is the command
        outCmd = *p++;
        outParamIsValid = false;
        outParam = 0.0f;

        // Skip whitespace after command
        while (*p && isspace((unsigned char)*p))
            ++p;

        // If there's anything left and it looks like a number, parse it with atof
        if (*p)
        {
            // Find first digit, sign, or decimal point
            char *numStart = p;
            // allow optional +/-
            if (*numStart == '+' || *numStart == '-')
                ++numStart;
            if (isdigit((unsigned char)*numStart) || *numStart == '.')
            {
                outParam = (float)atof(p);
                outParamIsValid = true;
            }
        }
        return true;
    }

    /**
     * Parse a single-char command optionally followed by up to TWO numeric parameters.
     * Accepts formats like:
     *   D 10 20
     *   D:10,20
     *   D,10,20
     *   D10 20
     *   D10,20
     *
     * Returns true if a command was parsed.
     * - If param1 exists: outP1Valid=true and outP1 set.
     * - If param2 exists: outP2Valid=true and outP2 set.
     */
    bool parseCmdWithUpToTwoFloats(const char *cmdStr,
                                   char &outCmd,
                                   float &outP1, bool &outP1Valid,
                                   float &outP2, bool &outP2Valid)
    {
        if (!cmdStr || cmdStr[0] == '\0')
            return false;

        // Copy into modifiable buffer and normalize separators to spaces
        char buf[SerialComs::MAX_CMD_LEN];
        strncpy(buf, cmdStr, sizeof(buf));
        buf[sizeof(buf) - 1] = '\0';

        for (char *p = buf; *p; ++p)
        {
            if (*p == ',' || *p == ':' || *p == '\t')
                *p = ' ';
        }

        // Skip leading whitespace
        char *p = buf;
        while (*p && isspace((unsigned char)*p))
            ++p;
        if (!*p)
            return false;

        // First non-space char is the command
        outCmd = *p++;
        outP1Valid = false;
        outP2Valid = false;
        outP1 = 0.0f;
        outP2 = 0.0f;

        // Helper: parse a float token if present at *p
        auto parseFloatAt = [](char *&ptr, float &outVal) -> bool
        {
            while (*ptr && isspace((unsigned char)*ptr))
                ++ptr;
            if (!*ptr)
                return false;

            // check if token could be a number
            char *s = ptr;
            if (*s == '+' || *s == '-')
                ++s;
            if (!(isdigit((unsigned char)*s) || *s == '.'))
                return false;

            outVal = (float)atof(ptr);

            // advance ptr to end of this token
            while (*ptr && !isspace((unsigned char)*ptr))
                ++ptr;
            return true;
        };

        // Parse up to two floats
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

private:
    HardwareSerial &_serial;
    char _buffer[MAX_CMD_LEN];
    uint8_t _bufLen;
    bool _hasCommand;
};

#endif // SERIAL_COMS_HPP
