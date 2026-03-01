#pragma once
// ColorSensor.hpp  (non-blocking, ISR pulse counting version)
// - No delays in update()
// - Uses attachInterrupt on the OUT pin to count pulses during short windows
// - Keeps your COLOR enum and getBlockColor() logic intact
// NOTES:
//  - This implementation assumes a single ColorSensor instance (ISR uses static pointer).
//  - Tweak measurementWindowMs and settleMs for faster/slower behavior (shorter -> noisier).

#include <Arduino.h>

struct ColorSensorPins
{
    uint8_t s0_pin;
    uint8_t s1_pin;
    uint8_t s2_pin;
    uint8_t s3_pin;
    uint8_t out_pin;
    uint8_t led_pin;
};

class ColorSensor
{
public:
    // max samples capacity (no dynamic allocation).
    static const uint8_t MAX_SAMPLES = 16;

    // State machine states used by update() -- non-blocking
    enum InternalState
    {
        STATE_SET_FILTER,
        STATE_WAIT_SETTLE,
        STATE_MEASURE_WINDOW,
        STATE_PROCESS_RESULTS
    };

    // Color selection for S2/S3 lines
    enum FILTER
    {
        FILTER_RED,
        FILTER_BLUE,
        FILTER_GREEN,
        FILTER_CLEAR
    };

    enum COLOR
    {
        BLUE,
        RED,
        YELLOW,
        NONE
    };

    ColorSensor(ColorSensorPins pins, uint8_t numSamples = 8,
                uint16_t settleMs = 5, uint16_t measurementWindowMs = 100)
        : _s0(pins.s0_pin),
          _s1(pins.s1_pin),
          _s2(pins.s2_pin),
          _s3(pins.s3_pin),
          _out(pins.out_pin),
          _led(pins.led_pin),
          _numSamples(numSamples ? min(numSamples, MAX_SAMPLES) : 1),
          _settleMs(settleMs),
          _measurementWindowMs(measurementWindowMs),
          _state(STATE_SET_FILTER),
          _currentFilter(FILTER_RED),
          _samplesTakenForFilter(0),
          _filterIndex(0),
          _lastStateChangeMs(0),
          _redFiltered(0.0f),
          _greenFiltered(0.0f),
          _blueFiltered(0.0f),
          _clearFiltered(0.0f)
    {
        for (uint8_t i = 0; i < MAX_SAMPLES; ++i)
        {
            _rawR[i] = _rawG[i] = _rawB[i] = _rawC[i] = 0.0f;
        }
        // register instance pointer for ISR
        s_instance = this;
    }

    // Call from setup()
    void init()
    {
        pinMode(_s0, OUTPUT);
        pinMode(_s1, OUTPUT);
        pinMode(_s2, OUTPUT);
        pinMode(_s3, OUTPUT);
        pinMode(_out, INPUT);
        pinMode(_led, OUTPUT);

        // stable frequency scaling (default in your original code)
        digitalWrite(_s0, HIGH);
        digitalWrite(_s1, LOW);

        // keep LED steady ON by default to avoid dimming artifacts.
        // If you need brightness control, use analogWrite on a PWM pin and keep
        // the duty constant (don't change it during measurements).
        digitalWrite(_led, HIGH);

        // Set initial filter pins
        selectFilter(FILTER_RED);

        // Setup ISR-based pulse counting on _out pin
        // attachInterrupt requires digitalPinToInterrupt(...)
        int irq = digitalPinToInterrupt(_out);
        if (irq != NOT_AN_INTERRUPT)
        {
            // clear counter and attach (ISR-based)
            _pulseCount = 0;
            _usePolling = false;
            attachInterrupt(irq, ColorSensor::isrRouter, CHANGE);
            Serial.print("ColorSensor: attached IRQ ");
            Serial.println(irq);
        }
        else
        {
            // fallback to polling mode — safe and header-only
            _usePolling = true;
            _pulseCount = 0;
            _lastPinState = digitalRead(_out);
            Serial.print("ColorSensor: pin ");
            Serial.print(_out);
            Serial.println(" not external-IRQ-capable; using polling mode.");
        }

        _lastStateChangeMs = millis();
        _state = STATE_SET_FILTER;
        _currentFilter = FILTER_RED;
        _samplesTakenForFilter = 0;
        _filterIndex = 0;
    }

    // Non-blocking update. Call often from loop()
    // The method advances the measurement state machine and when enough samples
    // have been collected for all 4 filters, it computes filtered values.
    void update()
    {
        unsigned long now = millis();

        switch (_state)
        {
        case STATE_SET_FILTER:
            // set the S2/S3 lines for the current filter, then move to settle
            selectFilter(_currentFilter);
            _lastStateChangeMs = now;
            _state = STATE_WAIT_SETTLE;
            break;

        case STATE_WAIT_SETTLE:
            // wait for sensor to settle (non-blocking)
            if ((now - _lastStateChangeMs) >= _settleMs)
            {
                // begin measurement window
                // reset pulse counter and record start time
                noInterrupts();
                _pulseCount = 0;
                unsigned long startMicros = micros(); // capture but not used here
                interrupts();
                _measurementWindowStartMs = now;
                _state = STATE_MEASURE_WINDOW;
            }
            break;

        case STATE_MEASURE_WINDOW:
            // If using polling, sample current pin state on every call to update()
            if (_usePolling)
            {
                uint8_t pinv = digitalRead(_out);
                if (pinv != _lastPinState)
                {
                    // edge detected (rising or falling). Count on RISING to be safe against bounce,
                    // or use CHANGE if you want both edges counted.
                    if (pinv == HIGH)
                    { // count rising edge only (less double-counting)
                        ++_pulseCount;
                    }
                    _lastPinState = pinv;
                }
            }

            // count pulses during measurementWindowMs (ISR increments _pulseCount)
            if ((now - _measurementWindowStartMs) >= _measurementWindowMs)
            {
                // sample complete; read and store count
                unsigned long count;
                noInterrupts();
                count = _pulseCount;
                // if polling mode, keep counting in subsequent windows (reset below)
                interrupts();

                // convert count to frequency (Hz): pulses / window_seconds
                float freqHz = 0.0f;
                if (_measurementWindowMs > 0)
                {
                    freqHz = (1000.0f * float(count)) / float(_measurementWindowMs);
                }

                // store sample into the appropriate buffer based on current filter
                switch (_currentFilter)
                {
                case FILTER_RED:
                    _rawR[_samplesTakenForFilter] = freqHz;
                    break;
                case FILTER_BLUE:
                    _rawB[_samplesTakenForFilter] = freqHz;
                    break;
                case FILTER_GREEN:
                    _rawG[_samplesTakenForFilter] = freqHz;
                    break;
                case FILTER_CLEAR:
                    _rawC[_samplesTakenForFilter] = freqHz;
                    break;
                }

                // reset counter for next sample window (important!)
                noInterrupts();
                _pulseCount = 0;
                interrupts();

                ++_samplesTakenForFilter;

                if (_samplesTakenForFilter >= _numSamples)
                {
                    // finished sampling this filter; advance to next filter (or process if done)
                    _samplesTakenForFilter = 0;
                    ++_filterIndex;
                    if (_filterIndex >= 4)
                    {
                        // done all filters -> compute results
                        _state = STATE_PROCESS_RESULTS;
                    }
                    else
                    {
                        // move to next filter (wrap order: RED, BLUE, GREEN, CLEAR)
                        _currentFilter = FILTER(_filterIndex);
                        _state = STATE_SET_FILTER;
                    }
                }
                else
                {
                    // take another sample for the same filter: go to WAIT_SETTLE again
                    _lastStateChangeMs = now;
                    _state = STATE_WAIT_SETTLE;
                }
            }
            break;

        case STATE_PROCESS_RESULTS:
        {
            // compute per-sample normalized ratios (channel/clear) then average those ratios.
            // This reduces the effect of clear-channel variation vs dividing the averages.
            float sumR = 0.0f;
            float sumG = 0.0f;
            float sumB = 0.0f;
            const float eps = 0.0001f;

            for (uint8_t i = 0; i < _numSamples; ++i)
            {
                float c = _rawC[i];
                if (c <= eps)
                {
                    // if clear is tiny/zero, treat ratio as zero to avoid inf/nan
                    // alternatively you might skip this sample (but we keep fixed window count)
                    // so we remain robust and sized the same as before.
                    // sum stays unchanged
                }
                else
                {
                    sumR += (_rawR[i] / c);
                    sumG += (_rawG[i] / c);
                    sumB += (_rawB[i] / c);
                }
            }

            _redFiltered = sumR / float(_numSamples);
            _greenFiltered = sumG / float(_numSamples);
            _blueFiltered = sumB / float(_numSamples);
            // keep clearFiltered as average clear frequency (useful debug/timebase)
            _clearFiltered = movingAverage(_rawC);

            // ready for new measurement cycle: reset indices
            _filterIndex = 0;
            _currentFilter = FILTER_RED;
            _samplesTakenForFilter = 0;
            _state = STATE_SET_FILTER;
        }
        break;
        }
    }

    // Getters for filtered values (units: Hz)
    inline float getRed() const { return _redFiltered; }
    inline float getGreen() const { return _greenFiltered; }
    inline float getBlue() const { return _blueFiltered; }
    inline float getClear() const { return _clearFiltered; }

    // Normalized ratio helpers (safe for divide-by-zero)
    inline float getRedNorm() const { return safeRatio(_redFiltered, _clearFiltered); }
    inline float getGreenNorm() const { return safeRatio(_greenFiltered, _clearFiltered); }
    inline float getBlueNorm() const { return safeRatio(_blueFiltered, _clearFiltered); }

    // Keep your getBlockColor() semantics exactly as requested
    COLOR getBlockColor()
    {
        float red = getRedNorm();
        float blue = getBlueNorm();
        float green = getGreenNorm();

        Serial.print("Red ");
        Serial.print(red);
        Serial.print("  Blue ");
        Serial.print(blue);
        Serial.print("  Green ");
        Serial.println(green);
        if (isBlueBlock(red, blue, green))
        {
            Serial.println("Blue block seen");
            return BLUE;
        }
        if (isRedBlock(red, blue, green))
        {
            Serial.println("Red block seen");
            return RED;
        }
        if (isYellowBlock(red, blue, green))
        {
            Serial.println("Yellow block seen");
            return YELLOW;
        }
        Serial.println("No Block seen");
        return NONE;
    }

    // LED control: keep steady (no flicker)
    inline void setLed(bool on) { digitalWrite(_led, on ? HIGH : LOW); }

    // Allow changing timing parameters at runtime
    void setSampleTiming(uint16_t settleMs, uint16_t windowMs)
    {
        _settleMs = settleMs;
        _measurementWindowMs = windowMs;
    }

private:
    // pins
    const uint8_t _s0, _s1, _s2, _s3, _out, _led;
    uint8_t _numSamples;

    // sample buffers
    float _rawR[MAX_SAMPLES];
    float _rawG[MAX_SAMPLES];
    float _rawB[MAX_SAMPLES];
    float _rawC[MAX_SAMPLES];

    // filtered results (Hz)
    float _redFiltered;
    float _greenFiltered;
    float _blueFiltered;
    float _clearFiltered;

    // measurement timing and state
    uint16_t _settleMs;
    uint16_t _measurementWindowMs;
    InternalState _state;
    FILTER _currentFilter;
    uint8_t _samplesTakenForFilter; // index within numSamples
    uint8_t _filterIndex;           // 0..3 mapping to RED,BLUE,GREEN,CLEAR
    unsigned long _lastStateChangeMs;
    unsigned long _measurementWindowStartMs;

    // ISR-based pulse counting (volatile)
    inline static ColorSensor *s_instance = nullptr; // single-instance ISR router (header-only, C++17)
    volatile unsigned long _pulseCount = 0;          // updated by ISR (or polling snapshot)
    bool _usePolling = false;                        // true when attachInterrupt not available or user wants polling
    uint8_t _lastPinState = LOW;                     // used by polling to detect edges

    // --------- color test logic preserved from your original code ----------
    inline bool isBlueBlock(float red, float blue, float green)
    {
        return (blue > .5f && blue < .7f) && (red > .1f && red < .20f) && (green > .25f && green < .40f);
    }
    inline bool isRedBlock(float red, float blue, float green)
    {
        return (blue > .2f && blue < .3f) && (red > .6f && red < .80f) && (green > .10f && green < .20f);
    }
    inline bool isYellowBlock(float red, float blue, float green)
    {
        return (blue > .2f && blue < .3f) && (red > .4f && red < .50f) && (green > .3f && green < .40f);
    }
    // --------------------------------------------------------------------

    inline static void isrRouter()
    {
        if (s_instance)
            s_instance->pulseIsr();
    }
    inline void pulseIsr()
    {
        // increment pulse count snapshot (fast, minimal work)
        ++_pulseCount;
    }

    inline void selectFilter(FILTER f)
    {
        // set S2/S3 according to desired filter mapping (matches original mapping)
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

    inline float movingAverage(const float *arr) const
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < _numSamples; ++i)
            sum += arr[i];
        return sum / float(_numSamples);
    }

    inline float safeRatio(float channel, float clear) const
    {
        if (clear <= 0.0001f)
            return 0.0f;
        return channel / clear;
    }
};