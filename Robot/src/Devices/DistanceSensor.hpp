#pragma once
// SharpGP2Y0A51.hpp
// Simple driver for Sharp GP2Y0A51SK0F (2-15 cm) for Arduino
// - uses two-point calibration for conversion V -> cm
// - EMA smoothing of measured voltage
// - example usage: call update() regularly, read getDistanceCm()

#include <Arduino.h>

/**
 * Calibration function maps voltage to distances for a given sensor
 */
typedef float (*VtoDfunction)(float voltage);

class SharpGP2Y0A51
{
public:
    

    // adcPin: analog pin (e.g., A0)
    // vref: ADC reference voltage in volts (default 5.0)
    // adcMax: ADC max reading (1023 for 10-bit AVR; 4095 for 12-bit)
    SharpGP2Y0A51(uint8_t adcPin,
                  VtoDfunction voltageToDistanceFunction = nullptr,
                  float vref = 5.0f,
                  uint16_t adcMax = 1023,
                  float emaAlpha = 0.03f)
        : _pin(adcPin),
          _vref(vref),
          _adcMax(adcMax),
          _alpha(constrain(emaAlpha, 0.01f, 0.99f)),
          _voltageEMA(0.0f),
          getDistanceFromVoltage(voltageToDistanceFunction),
          _k(0.0f),
          _b(0.0f),
          _isCalibrated(false)
    {
    }

    /**
     * Initialize the distance sensor
     */
    void init()
    {
        pinMode(_pin, INPUT);
        // seed EMA with first reading
        float v = readVoltageRaw();
        if (_voltageEMA == 0.0f)
            _voltageEMA = v;
    }

    /**
     * Get a reading from the distance sensor, and store internally
     */
    void update()
    {
        float v = readVoltageRaw();
        // Serial.println(v);
        // EMA smoothing
        
        _voltageEMA = _alpha * v + (1.0f - _alpha) * _voltageEMA;
        // Serial.print(_voltageEMA);
        // Serial.print("  ");
        _current_distance = getDistanceFromVoltage(_voltageEMA);
        // Serial.println(_current_distance);

    }

    /**
     * Return the last-smoothed voltage (volts)
     */
    float getVoltage() const { return _voltageEMA; }

    /**
     * Returns the most recent distance measurement from the sensor
     */
    float getDistanceCm() const
    {
        return _current_distance;
        
    }

    /**
     * Set smoothing for sensor
     */
    void setEMAalpha(float a) { _alpha = constrain(a, 0.01f, 0.99f); }

    /** 
     * Set expected physical min/max range for clamping
     * */
    void setRangeCm(float minCm, float maxCm)
    {
        _minRangeCm = minCm;
        _maxRangeCm = maxCm;
    }

    /**
     * Raw ADC reading (0..adcMax)
     */
    uint16_t readAdcRaw() const
    {
        return analogRead(_pin);
    }

    /**
     * Convert a raw ADC reading to voltage
     */
    float adcToVoltage(uint16_t raw) const
    {
        return ((float)raw * _vref) / (float)_adcMax;
    }

    /**
     * Read voltage once (no smoothing)
     */
    float readVoltageRaw() const
    {
        uint16_t raw = analogRead(_pin);
        return adcToVoltage(raw);
    }

private:
    // physical sensor range (defaults, can be overridden)
    float _minRangeCm = 2.0f;
    float _maxRangeCm = 15.0f;

    uint8_t _pin;
    float _vref;
    uint16_t _adcMax;
    float _alpha;
    volatile float _voltageEMA;
    float _current_distance = _minRangeCm;
    VtoDfunction getDistanceFromVoltage = nullptr;


    // calibration params for V = k*(1/d) + b
    float _k;
    float _b;
    bool _isCalibrated;

    
};
