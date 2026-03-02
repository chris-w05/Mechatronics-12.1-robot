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
        pinMode(_pin, INPUT);

    }

    // Call in your loop at least as fast as the sensor's update rate (~60 Hz typical)
    void update()
    {
        _isOn = digitalRead(_pin);
    }

    bool getReading(){
        return _isOn;
    }



private:
    uint8_t _pin;
    bool _isOn = false;


};