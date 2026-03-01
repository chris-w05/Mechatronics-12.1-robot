#pragma once
// ColorSensor.hpp
// Small, single-file Arduino-friendly driver for a TCS-style color sensor
// (the example Arduino sketch the user provided uses the same pins/signalling).
//
// - Header-only C++ class designed to match the style/architecture used across
//   the Mechatronics project (init(), update(), small public API).
// - Uses pulseIn() to measure the sensor output like your example sketch.
// - Simple moving-average filtering across N samples (configurable up to a safe max).
//
// Example usage:
//
//   // default pins match your sketch: s0=6,s1=5,s2=4,s3=3, out=2, led=13
//   ColorSensor cs;           // uses defaults, 8 samples
//   void setup() {
//     Serial.begin(57600);
//     cs.init();
//   }
//   void loop() {
//     if (!Serial.available()) return;
//     while (Serial.available()) Serial.read(); // flush
//     cs.update(); // take samples & update filtered values
//     Serial.print(cs.getRed(), 2); Serial.print(",\t");
//     Serial.print(cs.getGreen(), 2); Serial.print(",\t");
//     Serial.print(cs.getBlue(), 2); Serial.print(",\t");
//     Serial.println(cs.getClear(), 2);
//   }

#include <Arduino.h>

class ColorSensor
{
public:
    // max samples capacity (no dynamic allocation). Keep small for microcontrollers.
    static const uint8_t MAX_SAMPLES = 16;

    // Color selection for S2/S3 lines
    enum FILTER
    {
        FILTER_RED,
        FILTER_BLUE,
        FILTER_GREEN,
        FILTER_CLEAR
    };

    // Construct with pin assignment and number of samples for moving average.
    // numSamples will be clamped to [1, MAX_SAMPLES].
    ColorSensor(
        uint8_t s0_pin = 6,
        uint8_t s1_pin = 5,
        uint8_t s2_pin = 4,
        uint8_t s3_pin = 3,
        uint8_t out_pin = 2,
        uint8_t led_pin = 13,
        uint8_t numSamples = 8)
        : _s0(s0_pin),
          _s1(s1_pin),
          _s2(s2_pin),
          _s3(s3_pin),
          _out(out_pin),
          _led(led_pin),
          _numSamples(numSamples > 0 ? (numSamples > MAX_SAMPLES ? MAX_SAMPLES : numSamples) : 1)
    {
        // initialize arrays to zero
        for (uint8_t i = 0; i < MAX_SAMPLES; ++i)
        {
            _rawR[i] = _rawG[i] = _rawB[i] = _rawC[i] = 0.0f;
        }
        _redFiltered = _greenFiltered = _blueFiltered = _clearFiltered = 0.0f;
    }

    // Must be called from setup()
    void init()
    {
        pinMode(_s0, OUTPUT);
        pinMode(_s1, OUTPUT);
        pinMode(_s2, OUTPUT);
        pinMode(_s3, OUTPUT);
        pinMode(_out, INPUT);
        pinMode(_led, OUTPUT);

        // Default frequency-scaling: S0=HIGH, S1=LOW (matches your sketch)
        // This typically sets 100% output scaling on many TCS parts.
        digitalWrite(_s0, HIGH);
        digitalWrite(_s1, LOW);

        // turn LED on by default (matches your example)
        digitalWrite(_led, HIGH);
    }

    // Set the LED state (on/off)
    inline void setLed(bool on)
    {
        digitalWrite(_led, on ? HIGH : LOW);
    }

    // Directly set the frequency-scaling pins (convenience)
    inline void setFrequencyScaling(bool s0state, bool s1state)
    {
        digitalWrite(_s0, s0state ? HIGH : LOW);
        digitalWrite(_s1, s1state ? HIGH : LOW);
    }

    // One-shot update: takes _numSamples samples for each filter, updates filtered values.
    // This will take some time because it loops and calls pulseIn() multiple times.
    // Call from loop() when you want a new measurement (like your sketch).
    void update()
    {
        // read RED
        selectFilter(FILTER_RED);
        delay(10); // give sensor time to settle (matches original sketch)
        takeSamplesIntoArray(_rawR);

        // read BLUE
        selectFilter(FILTER_BLUE);
        delay(10);
        takeSamplesIntoArray(_rawB);

        // read GREEN
        selectFilter(FILTER_GREEN);
        delay(10);
        takeSamplesIntoArray(_rawG);

        // read CLEAR
        selectFilter(FILTER_CLEAR);
        delay(10);
        takeSamplesIntoArray(_rawC);

        // compute filtered values (moving average)
        _redFiltered = movingAverage(_rawR);
        _greenFiltered = movingAverage(_rawG);
        _blueFiltered = movingAverage(_rawB);
        _clearFiltered = movingAverage(_rawC);
    }

    // Getters for filtered values (units: pulse time as in example: sum of two pulseIn calls)
    inline float getRed() const { return _redFiltered; }
    inline float getGreen() const { return _greenFiltered; }
    inline float getBlue() const { return _blueFiltered; }
    inline float getClear() const { return _clearFiltered; }

    // Access to raw sample buffers (const pointers). Each buffer has _numSamples valid entries.
    inline const float *rawRed() const { return _rawR; }
    inline const float *rawGreen() const { return _rawG; }
    inline const float *rawBlue() const { return _rawB; }
    inline const float *rawClear() const { return _rawC; }

    // Query how many samples are used in the moving average
    inline uint8_t numSamples() const { return _numSamples; }

    // Convenience: set how many samples to average (clamped)
    void setNumSamples(uint8_t n)
    {
        _numSamples = (n == 0) ? 1 : (n > MAX_SAMPLES ? MAX_SAMPLES : n);
    }

private:
    // Pin assignments
    const uint8_t _s0, _s1, _s2, _s3, _out, _led;
    uint8_t _numSamples;

    // storage for raw samples (capacity MAX_SAMPLES, only first _numSamples used)
    float _rawR[MAX_SAMPLES];
    float _rawG[MAX_SAMPLES];
    float _rawB[MAX_SAMPLES];
    float _rawC[MAX_SAMPLES];

    // filtered values
    float _redFiltered;
    float _greenFiltered;
    float _blueFiltered;
    float _clearFiltered;

    // Select color filter via S2/S3
    inline void selectFilter(FILTER f)
    {
        switch (f)
        {
        case FILTER_RED:
            digitalWrite(_s2, LOW);
            digitalWrite(_s3, LOW);
            break;
        case FILTER_BLUE:
            digitalWrite(_s2, LOW);
            digitalWrite(_s3, HIGH);
            break;
        case FILTER_GREEN:
            digitalWrite(_s2, HIGH);
            digitalWrite(_s3, HIGH);
            break;
        case FILTER_CLEAR:
            digitalWrite(_s2, HIGH);
            digitalWrite(_s3, LOW);
            break;
        }
    }

    // Fill provided buffer with _numSamples samples from the OUT pin.
    // Each entry is the same metric used in your sketch:
    //    pulseIn(out, LOW) + pulseIn(out, HIGH)
    inline void takeSamplesIntoArray(float *buffer)
    {
        // guard (shouldn't happen, but be safe)
        if (!buffer)
            return;

        for (uint8_t i = 0; i < _numSamples; ++i)
        {
            buffer[i] = readPulse();
            // small gap between successive pulse measurements helps stability
            // but keep short so update() doesn't become too slow
            delayMicroseconds(200);
        }
    }

    // The same readPulse() implementation from the sketch
    inline float readPulse()
    {
        // pulseIn returns unsigned long; convert to float
        unsigned long a = pulseIn(_out, LOW, 30000UL); // add timeout to avoid lockups
        unsigned long b = pulseIn(_out, HIGH, 30000UL);
        return float(a + b);
    }

    // Simple moving average across the active window
    inline float movingAverage(const float *arr) const
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < _numSamples; ++i)
        {
            sum += arr[i];
        }
        return sum / float(_numSamples);
    }
};