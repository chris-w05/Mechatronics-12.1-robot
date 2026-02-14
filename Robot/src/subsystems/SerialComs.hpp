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

private:
    HardwareSerial &_serial;
    char _buffer[MAX_CMD_LEN];
    uint8_t _bufLen;
    bool _hasCommand;
};

#endif // SERIAL_COMS_HPP
