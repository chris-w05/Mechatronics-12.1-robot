#ifndef SOLENOID_HPP
#define SOLENOID_HPP

#include <Arduino.h>

/**
 * Simple solenoid control class
 *
 * Controls a solenoid using a digital output pin.
 * HIGH = energized (on)
 * LOW  = de-energized (off)
 */
class Solenoid
{
public:
    /**
     * @param pin Digital output pin connected to the solenoid driver
     * @param activeHigh Whether HIGH turns the solenoid on (default true)
     */
    Solenoid(uint8_t pin, bool activeHigh = true)
        : _pin(pin), _activeHigh(activeHigh), _state(false)
    {
    }

    /**
     * Must be called once in setup()
     */
    void init()
    {
        pinMode(_pin, OUTPUT);
        off(); // ensure safe startup
    }

    /**
     * Energize the solenoid
     */
    void on()
    {
        digitalWrite(_pin, _activeHigh ? HIGH : LOW);
        _state = true;
    }

    /**
     * De-energize the solenoid
     */
    void off()
    {
        digitalWrite(_pin, _activeHigh ? LOW : HIGH);
        _state = false;
    }

    /**
     * Toggle solenoid state
     */
    void toggle()
    {
        if (_state)
            off();
        else
            on();
    }

    /**
     * @return true if solenoid is energized
     */
    bool isOn() const
    {
        return _state;
    }

private:
    uint8_t _pin;
    bool _activeHigh;
    bool _state;
};

#endif // SOLENOID_HPP
