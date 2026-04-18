/**
 * @file EncoderWrapper.hpp
 * @brief Quadrature encoder wrapper providing position, velocity, and acceleration.
 *
 * Wraps the PJRC `Encoder` library and adds:
 * - Configurable direction flip.
 * - IIR-filtered velocity and acceleration estimates computed from tick deltas.
 *
 * Units returned are raw encoder ticks and ticks/second; callers apply the
 * appropriate ticks-to-physical-units scale factor (see #DRIVETRAIN_TICKS_TO_IN).
 */
#pragma once
#include <Arduino.h>
#include <Encoder.h>

/**
 * @brief Quadrature encoder driver with velocity and acceleration estimation.
 *
 * Call `init()` once in setup and `update()` every control cycle.  All
 * motion quantities are filtered through a configurable IIR filter to reduce
 * quantisation noise.
 */
class EncoderWrapper
{
public:
    EncoderWrapper(uint8_t pinA, uint8_t pinB)
        : _pinA(pinA), _pinB(pinB), encoder(pinA, pinB)
    {
    }

    void init()
    {
        pinMode(_pinA, INPUT_PULLUP); // try enabling pullups
        pinMode(_pinB, INPUT_PULLUP);
        encoder.write(0); // baseline
        lastMicros = micros();
        count = encoder.read();
        lastCount = count;
        velocity = 0;
        lastVelocity = 0;
        acceleration = 0;
    }

    /**
     * Reverses the reading of the encoder
     */
    void flipDirection(){ isReversed = !isReversed;}

    /**
     * Updates the position, velocity, and aacceleration of the encoder. 
     */
    void update()
    {
        unsigned long now = micros();
        count = encoder.read();
        
        unsigned long dt = now - lastMicros;
        if (dt > 0)
        {
            long dCount = count - lastCount;
            velocity = (float)dCount * (1000000.0 / dt);
            velocity = filter(velocity, lastVelocity);
            float dVelocity = velocity - lastVelocity;
            acceleration = dVelocity * (1000.0 / dt);
            acceleration = filter(dVelocity, lastAcceleration);
            lastMicros = now;
            lastCount = count;
            lastVelocity = velocity;
            lastAcceleration = acceleration;
        }
    }

    /**
     * Gets the current position of the encoder
     * 
     * @returns The position of the encoder (in ticks - 64/revolution)
     */
    int32_t getCount() { return isReversed ? -count : count; }

    /**
     * Gets teh current velocity of the encoder
     * 
     * @returns The velocity of the encoder in ticks/second
     */
    float getVelocity() { return isReversed ? -velocity : velocity; }

    /**
     * Gets the current acceleration of the encoder
     * 
     * @returns The acceleration of the encoder in ticks / second squared
     */
    float getAcceleration() { return isReversed ? -acceleration : acceleration; }

private:
    const float filter(float newValue, float oldValue) {
        return alpha * newValue + (1-alpha) * oldValue;
    }

    bool isReversed;      ///< When true, all count/velocity/acceleration values are negated
    uint8_t _pinA;        ///< Encoder channel A pin
    uint8_t _pinB;        ///< Encoder channel B pin
    Encoder encoder;      ///< Underlying PJRC Encoder object
    int32_t count     = 0; ///< Latest absolute tick count
    int32_t lastCount = 0; ///< Tick count on previous update() call
    unsigned long lastMicros = 0; ///< Timestamp of previous update() call (µs)
    unsigned long INTERVAL   = 100; ///< Minimum update interval (µs) — currently unused
    float velocity      = 0;  ///< Filtered velocity (ticks/s)
    float lastVelocity  = 0;  ///< Previous velocity estimate (for IIR filter)
    float lastAcceleration = 0; ///< Previous acceleration estimate (for IIR filter)
    float acceleration  = 0;  ///< Filtered acceleration (ticks/s²)

    float alpha = 0.5f; ///< IIR filter coefficient for velocity/acceleration (0 = strongest, 1 = off)
};
