#pragma once
// HallSensor.hpp
// Header-only Arduino-friendly driver for a generic digital Hall-effect sensor
//
// - Matches the init()/update() style used across the Mechatronics project.
// - Polling-based edge detection with debounce (no ISR required).
// - Counts pulses, measures last pulse interval, computes frequency (Hz) and RPM.
// - Small, predictable memory footprint (suitable for microcontrollers).
//
// Example usage:
//
//   HallSensor wheelHall(2, true /*activeLow*/, true /*enablePullup*/, 5 /*debounce ms*/, 1 /*pulsesPerRev*/);
//   void setup() {
//     Serial.begin(115200);
//     wheelHall.init();
//   }
//   void loop() {
//     wheelHall.update(); // call frequently (every loop)
//     if (wheelHall.pulseDetected()) {
//       Serial.print("Count: "); Serial.print(wheelHall.getCount());
//       Serial.print("  RPM: "); Serial.println(wheelHall.getRPM(), 2);
//     }
//   }

#include <Arduino.h>
#include <climits>

class HallSensor
{
public:
    // Construct a Hall sensor object
    // pin              : digital pin connected to sensor output
    // activeLow        : true if sensor output is LOW when active (typical open-collector hall)
    // enablePullup     : enable internal pull-up during init()
    // debounceMs       : debounce window in milliseconds for edge detection (>=0)
    // pulsesPerRev     : number of pulses produced per mechanical revolution (used for RPM)
    HallSensor(uint8_t pin,
               bool activeLow = true,
               bool enablePullup = true,
               uint16_t debounceMs = 5,
               uint16_t pulsesPerRev = 1)
        : _pin(pin),
          _activeLevel(activeLow ? LOW : HIGH),
          _enablePullup(enablePullup),
          _debounceMs(debounceMs),
          _pulsesPerRev(pulsesPerRev),
          _count(0),
          _lastRawState(HIGH),
          _stableState(HIGH),
          _lastDebounceTime(0),
          _lastEdgeTimeMicros(0),
          _lastPulseIntervalMicros(0),
          _edgeDetected(false)
    {
    }

    // Must be called from setup()
    void init()
    {
        if (_enablePullup)
        {
            pinMode(_pin, INPUT_PULLUP);
        }
        else
        {
            pinMode(_pin, INPUT);
        }
        // initialize states
        _lastRawState = digitalRead(_pin);
        _stableState = _lastRawState;
        _lastDebounceTime = millis();
        _lastEdgeTimeMicros = 0;
        _lastPulseIntervalMicros = 0;
        _edgeDetected = false;
    }

    // Call frequently from loop(). This performs debounced edge detection and bookkeeping.
    // Uses millis() for debounce timing and micros() for pulse interval resolution.
    void update()
    {
        _edgeDetected = false; // reset per-cycle flag

        int raw = digitalRead(_pin);

        // If raw reading differs from lastRawState, reset debounce timer
        if (raw != _lastRawState)
        {
            _lastDebounceTime = millis();
            _lastRawState = raw;
        }

        // If the reading has been stable longer than debounce window, treat it as stable
        if ((millis() - _lastDebounceTime) >= _debounceMs)
        {
            if (raw != _stableState)
            {
                // stable state changed -> an edge occurred
                _stableState = raw;
                handleEdge(_stableState);
            }
        }
    }

    // Return true if a new edge (active transition) was detected since last update()
    // This is helpful for event-driven logic in the main loop.
    inline bool pulseDetected() const { return _edgeDetected; }

    // Get total pulse count (monotonic since init or last reset)
    inline unsigned long getCount() const { return _count; }

    // Reset pulse count to zero
    inline void resetCount() { _count = 0; }

    // Get last pulse interval in milliseconds (float). Returns 0 if not enough pulses yet.
    inline float getLastPulseIntervalMs() const { return float(_lastPulseIntervalMicros) / 1000.0f; }

    // Get measured frequency in Hz (based on last interval). Returns 0 if no interval recorded yet.
    inline float getFrequencyHz() const
    {
        if (_lastPulseIntervalMicros == 0)
            return 0.0f;
        return 1e6f / float(_lastPulseIntervalMicros);
    }

    // Get RPM computed from pulsesPerRev. Returns 0 if no interval recorded yet.
    inline float getRPM() const
    {
        float hz = getFrequencyHz();
        if (hz <= 0.0f)
            return 0.0f;
        return (hz * 60.0f) / float(_pulsesPerRev);
    }

    // Convenience: returns whether the current stable state is active
    inline bool isActive() const { return _stableState == _activeLevel; }

    // Adjust configuration at runtime:
    inline void setDebounceMs(uint16_t ms) { _debounceMs = ms; }
    inline uint16_t getDebounceMs() const { return _debounceMs; }
    inline void setPulsesPerRev(uint16_t p) { _pulsesPerRev = (p == 0 ? 1 : p); }
    inline uint16_t getPulsesPerRev() const { return _pulsesPerRev; }

private:
    // pin + config
    const uint8_t _pin;
    const int _activeLevel;
    const bool _enablePullup;

    // debounce and mechanical conversion
    uint16_t _debounceMs;
    uint16_t _pulsesPerRev;

    // counters & state
    unsigned long _count;

    // raw/debounced state tracking
    int _lastRawState;
    int _stableState;
    unsigned long _lastDebounceTime; // millis()

    // pulse timing (high-resolution)
    unsigned long _lastEdgeTimeMicros;      // micros() of previous detected active edge
    unsigned long _lastPulseIntervalMicros; // interval between last two active edges (micros)

    // transient flag set during update() on new active edge
    bool _edgeDetected;

    // Called on any stable state change. We only treat active transitions as pulses.
    inline void handleEdge(int stableState)
    {
        unsigned long nowMicros = micros();

        // Determine if this transition is an active edge (inactive->active)
        // If active level is LOW, then active transition is from HIGH -> LOW
        bool becameActive = (stableState == _activeLevel);

        if (becameActive)
        {
            // increment count
            ++_count;
            // update interval
            if (_lastEdgeTimeMicros != 0)
            {
                // compute non-zero interval (micros)
                unsigned long interval = (nowMicros >= _lastEdgeTimeMicros)
                                             ? (nowMicros - _lastEdgeTimeMicros)
                                             : (ULONG_MAX - _lastEdgeTimeMicros + nowMicros + 1UL); // handle micros() overflow
                _lastPulseIntervalMicros = interval;
            }
            // update last edge time
            _lastEdgeTimeMicros = nowMicros;
            // set detected flag for this loop
            _edgeDetected = true;
        }
        else
        {
            // falling/idle edge - we don't count it as a pulse (only count active transitions),
            // but we still update nothing else.
        }
    }
};