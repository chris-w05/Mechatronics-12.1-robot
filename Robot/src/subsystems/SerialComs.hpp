/**
 * @file SerialComs.hpp
 * @brief Newline-delimited ASCII serial command receiver subsystem.
 *
 * Wraps a `HardwareSerial` port and buffers incoming characters until a
 * newline (`\n`) or carriage-return (`\r`) is received, then exposes the
 * complete command string through `getCommand()` / `getCommandChar()`.
 *
 * Two parser helpers (`parseCmdWithOptionalFloat`, `parseCmdWithUpToTwoFloats`)
 * decode commands of the form `<letter>[sep]<number>[sep]<number>` where sep
 * may be a space, comma, or colon.
 */
#ifndef SERIAL_COMS_HPP
#define SERIAL_COMS_HPP

#include <Arduino.h>

/**
 * @brief Newline-delimited ASCII command receiver for Arduino-to-Arduino comms.
 *
 * Commands are ASCII strings terminated by `\n` (or `\r`, normalised to `\n`).
 * Buffer overflow silently discards the malformed frame and resets for the
 * next one.
 *
 * @code
 *   SerialComs com(Serial1);
 *   com.init();
 *   // in loop:
 *   com.update();
 *   if (com.hasCommand()) Serial.println(com.getCommand());
 * @endcode
 */
class SerialComs : public Subsystem
{
public:
    static const uint8_t MAX_CMD_LEN = 32; ///< Maximum bytes in a single command (including NUL)

    /**
     * @brief Construct the SerialComs subsystem.
     * @param serialPort  Reference to the `HardwareSerial` port to listen on.
     *                    The caller is responsible for calling `begin()` on it.
     */
    SerialComs(HardwareSerial &serialPort)
        : _serial(serialPort), _bufLen(0), _hasCommand(false)
    {
        _buffer[0] = '\0';
    }

    /** @brief Initialise the subsystem (call once in `setup()`). */
    void init() override
    {
        Serial.println("Serial communications initialized");
        // Nothing else required here; the HardwareSerial should be begun by the caller.
    }

    /**
     * @brief Non-blocking read — call every loop tick.
     *
     * Accumulates incoming characters.  Sets `_hasCommand` and returns after
     * receiving one complete newline-terminated frame.
     */
    void update() override
    {
        // Serial.println("Updating coms");
        while (_serial.available() > 0)
        {
            char c = (char)_serial.read();

            // Treat CR as command terminator too (handles monitors configured for CR-only).
            if (c == '\r')
                c = '\n';

            // End of command
            if (c == '\n')
            {
                if (_bufLen == 0)
                    continue; // ignore blank lines

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

    /** @brief @return `true` if a complete command is waiting to be read. */
    bool hasCommand() const
    {
        return _hasCommand;
    }

    /**
     * @brief Return the latest command string and clear the pending flag.
     * @return Pointer to the internal null-terminated command buffer.
     * @note The pointer is valid until the next call to `update()`.
     */
    const char *getCommand()
    {
        _hasCommand = false;
        return _buffer;
    }

    /**
     * @brief Return the first character of the latest command and clear the flag.
     * @return First character of the command, or `0` if no command was pending.
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
     * @brief Transmit a message over the serial port (appends a newline).
     * @param msg  Null-terminated string to send.
     */
    void send(const char *msg)
    {
        _serial.println(msg);
    }

    /** @brief Stop the subsystem (no-op for SerialComs). */
    virtual void stop() override {}

    /**
     * @brief Parse a command letter followed by an optional float parameter.
     *
     * Accepts separators: space, comma, colon, or no separator.
     * Examples: `"D 10"`, `"D:10"`, `"D,10"`, `"D10"`.
     *
     * @param cmdStr        Null-terminated command string to parse.
     * @param outCmd        Output: the leading command character.
     * @param outParam      Output: the parsed float parameter (0 if absent).
     * @param outParamIsValid Output: `true` if a numeric parameter was found.
     * @return `true` if at least a command character was successfully parsed.
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
     * @brief Parse a command letter followed by up to two optional float parameters.
     *
     * Accepts separators: space, comma, colon, or tab.
     * Examples: `"D 10 20"`, `"D:10,20"`, `"D10,20"`.
     *
     * @param cmdStr    Null-terminated command string to parse.
     * @param outCmd    Output: the leading command character.
     * @param outP1     Output: first numeric parameter (0 if absent).
     * @param outP1Valid Output: `true` if the first parameter was parsed.
     * @param outP2     Output: second numeric parameter (0 if absent).
     * @param outP2Valid Output: `true` if the second parameter was parsed.
     * @return `true` if at least a command character was successfully parsed.
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
    HardwareSerial &_serial; ///< Hardware serial port used for TX and RX
    char _buffer[MAX_CMD_LEN]; ///< Accumulation buffer for the in-progress command
    uint8_t _bufLen;           ///< Number of bytes currently in `_buffer`
    bool _hasCommand;          ///< True when a complete command is ready to be read
};

#endif // SERIAL_COMS_HPP
