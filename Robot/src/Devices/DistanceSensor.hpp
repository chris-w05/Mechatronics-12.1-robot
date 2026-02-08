#pragma once
// SharpGP2Y0A51.hpp
// Simple driver for Sharp GP2Y0A51SK0F (2-15 cm) for Arduino
// - uses two-point calibration for conversion V -> cm
// - EMA smoothing of measured voltage
// - example usage: call update() regularly, read getDistanceCm()

#include <Arduino.h>

class SharpGP2Y0A51
{
public:
    // adcPin: analog pin (e.g., A0)
    // vref: ADC reference voltage in volts (default 5.0)
    // adcMax: ADC max reading (1023 for 10-bit AVR; 4095 for 12-bit)
    SharpGP2Y0A51(uint8_t adcPin,
                  float vref = 5.0f,
                  uint16_t adcMax = 1023,
                  float emaAlpha = 0.2f)
        : _pin(adcPin),
          _vref(vref),
          _adcMax(adcMax),
          _alpha(constrain(emaAlpha, 0.01f, 0.99f)),
          _voltageEMA(0.0f),
          _k(0.0f),
          _b(0.0f),
          _isCalibrated(false)
    {
    }

    // Call in setup()
    void init()
    {
        pinMode(_pin, INPUT);
        // seed EMA with first reading
        float v = readVoltageRaw();
        if (_voltageEMA == 0.0f)
            _voltageEMA = v;
    }

    // Call in your loop at least as fast as the sensor's update rate (~60 Hz typical)
    void update()
    {
        float v = readVoltageRaw();
        // EMA smoothing
        _voltageEMA = _alpha * v + (1.0f - _alpha) * _voltageEMA;
    }

    // Return the last-smoothed voltage (volts)
    float getVoltage() const { return _voltageEMA; }

    // Convert to distance in centimeters using current calibration
    // If not calibrated, returns -1 and you should call calibrate() first or setCalibration()
    float getDistanceCm() const
    {
        if (!_isCalibrated)
            return -1.0f;
        float V = _voltageEMA;
        // avoid divide-by-zero or invalid range
        if (V <= _b + 1e-6f)
            return 9999.0f;
        float d = _k / (V - _b);
        // clamp to sensor range (user can change these constants)
        d = constrain(d, _minRangeCm, _maxRangeCm);
        return d;
    }

    // Provide k and b directly (units: k in cm*V, b in V)
    // Model: V = k * (1/d) + b  ->  d = k / (V - b)
    void setCalibrationParams(float k, float b)
    {
        _k = k;
        _b = b;
        _isCalibrated = true;
    }

    // Two-point calibration:
    // supply two measured distances (cm) and the corresponding measured voltages (V)
    // Example usage: place a white sheet at 5 cm and 12 cm, read voltages (use getVoltage())
    // then call calibrateTwoPoints(5.0, v_at_5, 12.0, v_at_12)
    bool calibrateTwoPoints(float d1_cm, float v1_v, float d2_cm, float v2_v)
    {
        // Validate inputs
        if (d1_cm <= 0 || d2_cm <= 0)
            return false;
        if (fabs(v1_v - v2_v) < 1e-6f)
            return false; // can't calibrate with identical voltages

        // From V = k*(1/d) + b
        // Solve linear system for k and b:
        // v1 = k*(1/d1) + b
        // v2 = k*(1/d2) + b
        float inv1 = 1.0f / d1_cm;
        float inv2 = 1.0f / d2_cm;

        _k = (v1_v - v2_v) / (inv1 - inv2);
        _b = v1_v - _k * inv1;

        _isCalibrated = true;
        return true;
    }

    // Set smoothing alpha (0..1). Lower = smoother/longer memory
    void setEMAalpha(float a) { _alpha = constrain(a, 0.01f, 0.99f); }

    // Set expected physical min/max range for clamping
    void setRangeCm(float minCm, float maxCm)
    {
        _minRangeCm = minCm;
        _maxRangeCm = maxCm;
    }

    // Raw ADC reading (0..adcMax)
    uint16_t readAdcRaw() const
    {
        return analogRead(_pin);
    }

    // Convert a raw ADC reading to voltage
    float adcToVoltage(uint16_t raw) const
    {
        return ((float)raw * _vref) / (float)_adcMax;
    }

    // Average a bunch of raw voltage samples (helper for calibration)
    float averageRawVoltage(uint16_t samples = 10, uint16_t delayMsBetween = 10) const
    {
        uint32_t acc = 0;
        for (uint16_t i = 0; i < samples; ++i)
        {
            acc += analogRead(_pin);
            if (delayMsBetween)
                delay(delayMsBetween);
        }
        float avgRaw = (float)acc / (float)samples;
        return adcToVoltage((uint16_t)round(avgRaw));
    }

    // Read voltage once (no smoothing)
    float readVoltageRaw() const
    {
        uint16_t raw = analogRead(_pin);
        return adcToVoltage(raw);
    }

    // For debugging: return whether calibrated
    bool isCalibrated() const { return _isCalibrated; }

private:
    uint8_t _pin;
    float _vref;
    uint16_t _adcMax;
    float _alpha;
    volatile float _voltageEMA;

    // calibration params for V = k*(1/d) + b
    float _k;
    float _b;
    bool _isCalibrated;

    // physical sensor range (defaults, can be overridden)
    float _minRangeCm = 2.0f;
    float _maxRangeCm = 15.0f;
};
