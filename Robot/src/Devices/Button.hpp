#pragma once
#include <Arduino.h>

class Button
{
public:
    // adcPin: analog pin (e.g., A0)
    // vref: ADC reference voltage in volts (default 5.0)
    // adcMax: ADC max reading (1023 for 10-bit AVR; 4095 for 12-bit)
    Button(uint8_t pin)
        : _pin(pin)
    {
    }

    // Call in setup()
    void init()
    {
        pinMode(_pin, INPUT_PULLUP);
    }

    // Call in your loop at least as fast as the sensor's update rate (~60 Hz typical)
    void update()
    {
        //XOR operator for reversed case
        _isOn = digitalRead(_pin) ^ reversed;
    }

    bool getReading(){
        return _isOn;
    }

    void reverse(){
        reversed = !reversed;
    }



private:
    uint8_t _pin;
    bool _isOn = false;
    bool reversed = false;
};