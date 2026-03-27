/**
 * @file DistanceSensor.hpp
 * @brief Driver for the Sharp GP2Y0A51SK0F short-range IR distance sensor (2–15 cm).
 *
 * Reads an analog voltage from the IR sensor, applies exponential moving
 * average (EMA) smoothing, and converts the smoothed voltage to a distance
 * using a user-supplied calibration function.
 *
 * Typical usage:
 * @code
 * SharpGP2Y0A51 dist(A4, distanceSensor_VoltageToDistance);
 * dist.init();
 * // in loop:
 * dist.update();
 * float cm = dist.getDistanceCm();
 * @endcode
 */
#pragma once
// SharpGP2Y0A51.hpp
// Simple driver for Sharp GP2Y0A51SK0F (2-15 cm) for Arduino
// - uses two-point calibration for conversion V -> cm
// - EMA smoothing of measured voltage
// - example usage: call update() regularly, read getDistanceCm()

#include <Arduino.h>

/**
 * @brief Function-pointer type for a voltage-to-distance mapping.
 * @param voltage Smoothed sensor output (V).
 * @return        Distance (cm).
 */
typedef float (*VtoDfunction)(float voltage);

/**
 * @brief Sharp GP2Y0A51 analog IR distance sensor driver.
 *
 * Reads the sensor ADC value, converts to voltage, applies EMA smoothing,
 * and runs the result through a user-supplied calibration function to
 * produce a distance in centimetres.
 */
class SharpGP2Y0A51
{
public:
    

    /**
     * @brief Construct the sensor driver.
     * @param adcPin                    Arduino analog pin (e.g. A4).
     * @param voltageToDistanceFunction Calibration function mapping voltage → cm.
     * @param vref                      ADC reference voltage in volts (default 5.0).
     * @param adcMax                    ADC full-scale count (1023 for 10-bit AVR).
     * @param emaAlpha                  EMA smoothing factor.  Closer to 1.0 = less smoothing.
     */
    SharpGP2Y0A51(uint8_t adcPin,
                  VtoDfunction voltageToDistanceFunction = nullptr,
                  float vref = 5.0f,
                  uint16_t adcMax = 1023,
                  float emaAlpha = 0.1f)
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
     * Returns the most recent distance measurement from the sensor in inches
     */
    float getDistanceIn() const{
        return _current_distance / 2.54;
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
