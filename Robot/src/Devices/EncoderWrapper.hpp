#pragma once
#include <Arduino.h>
#include <Encoder.h>

class EncoderWrapper
{
public:
    // adcPin: analog pin (e.g., A0)
    // vref: ADC reference voltage in volts (default 5.0)
    // adcMax: ADC max reading (1023 for 10-bit AVR; 4095 for 12-bit)
    EncoderWrapper(uint8_t pinA, uint8_t pinB)
        :  _pinA(pinA), _pinB(pinB), encoder(pinA, pinB)
    {
    }

    // Call in setup()
    void init()
    {
        encoder.begin(_pinA, _pinB);
    }

    // Call in your loop at least as fast as the sensor's update rate (~60 Hz typical)
    void update()
    {
        count = encoder.read();

    }

    bool getReading()
    {
        return _isOn;
    }

    int32_t read(){
        return count;
    }

    double getVelocity(){
        return 0;
    }

private:
    uint8_t _pinA;
    uint8_t _pinB;
    Encoder encoder;

    int32_t count = 0;
    bool _isOn = false;
};