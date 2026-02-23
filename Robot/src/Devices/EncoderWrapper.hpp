#pragma once
#include <Arduino.h>
#include <Encoder.h>

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
        lastMicros = millis();
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
    const float filter( float newValue, float oldValue){
        return alpha * newValue + (1-alpha) * oldValue;
    }

    bool isReversed;
    uint8_t _pinA;
    uint8_t _pinB;
    Encoder encoder;
    int32_t count = 0;
    int32_t lastCount = 0;
    unsigned long lastMicros = 0;
    unsigned long INTERVAL = 100;
    float velocity = 0;
    float lastVelocity = 0;
    float lastAcceleration = 0;
    float acceleration = 0;

    /** IIR Filter constant for velocity and acceleration  */
    float alpha = .07;
};
