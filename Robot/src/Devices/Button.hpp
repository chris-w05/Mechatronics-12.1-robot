/**
 * @file Button.hpp
 * @brief Simple digital button (or limit-switch) driver.
 *
 * Wraps a single `INPUT_PULLUP` digital pin.  Reading is active-HIGH after
 * the internal pull-up is applied; call `reverse()` to invert the polarity
 * (useful for normally-closed switches wired to GND).
 */
#pragma once
#include <Arduino.h>

/**
 * @brief Digital button / limit-switch driver.
 *
 * Call `init()` once in setup, then call `update()` every loop and read the
 * debounced state with `getReading()`.
 */
class Button
{
public:
    /**
     * @brief Construct a button on a given pin.
     * @param pin  Arduino digital pin number.
     */
    Button(uint8_t pin)
        : _pin(pin)
    {
    }

    /**
     * Set pin mode for button
     */
    void init()
    {
        pinMode(_pin, INPUT_PULLUP);
    }

    /**
     * Read and store value from button
     */
    void update()
    {
        //XOR operator for reversed case
        _isOn = digitalRead(_pin) ^ reversed;
    }

    /**
     * Get reading from button
     */
    bool getReading(){
        return _isOn;
    }

    void reverse(){
        reversed = !reversed;
    }



private:
    uint8_t _pin;         ///< Arduino pin number
    bool _isOn    = false; ///< Debounced (instantaneous) reading, after polarity inversion
    bool reversed = false; ///< When true, reading is XOR'd to invert active polarity
};