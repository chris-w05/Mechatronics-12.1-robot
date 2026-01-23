#ifndef SERIAL_COMS_HPP
#define SERIAL_COMS_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>
/**
 * SerialComs
 *
 * Simple command receiver for Arduino-to-Arduino communication.
 * Commands are newline-terminated ASCII strings.
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
    void init()
    {
        
    }

    /**
     * Call periodically in loop()
     * Non-blocking.
     */
    void update()
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
                // Overflow → reset buffer
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
     * Get the latest command.
     * Calling this clears the command flag.
     */
    const char *getCommand()
    {
        _hasCommand = false;
        return _buffer;
    }

    /**
     * Send a message back
     */
    void send(const char *msg)
    {
        _serial.println(msg);
    }

    virtual void stop() {}

private:
    SoftwareSerial &_serial;
    char _buffer[MAX_CMD_LEN];
    uint8_t _bufLen;
    bool _hasCommand;
};

#endif // SERIAL_COMS_HPP
