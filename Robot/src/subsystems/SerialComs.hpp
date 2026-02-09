#ifndef SERIAL_COMS_HPP
#define SERIAL_COMS_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>

/**
 * SerialComs
 *
 * Simple command receiver for Arduino-to-Arduino communication.
 * Commands are newline-terminated ASCII strings.
 *
 * Improvements:
 * - getCommandChar() returns the first character of the latest newline-terminated command
 *   and clears the command flag. This makes single-character commands easy to use.
 */
class SerialComs : public Subsystem
{
public:
    static const uint8_t MAX_CMD_LEN = 32;

    /**
     * @param serial Reference to a SoftwareSerial (Serial, Serial1, etc)
     */
    SerialComs(SoftwareSerial &serial)
        : _serial(serial), _bufLen(0), _hasCommand(false)
    {
    }

    /**
     * Call once in setup()
     */
    void init() override
    {
        Serial.println("Serial communications initialized");
        // nothing special to init here
    }

    /**
     * Call periodically in loop()
     * Non-blocking. Buffers up to MAX_CMD_LEN-1 chars and sets a flag when newline ('\n') is seen.
     */
    void update() override
    {
        while (_serial.available() > 0)
        {
            char c = _serial.read();

            // Ignore carriage return
            if (c == '\r')
                continue;

            // End of command
            if (c == '\n')
            {
                _buffer[_bufLen] = '\0';
                _hasCommand = true;
                _bufLen = 0;
                return; // process one command per update
            }

            // Add character if space remains
            if (_bufLen < MAX_CMD_LEN - 1)
            {
                _buffer[_bufLen++] = c;
            }
            else
            {
                // Overflow → reset buffer to avoid partial/malformed commands
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
    SoftwareSerial &_serial;
    char _buffer[MAX_CMD_LEN];
    uint8_t _bufLen;
    bool _hasCommand;
};

#endif // SERIAL_COMS_HPP
