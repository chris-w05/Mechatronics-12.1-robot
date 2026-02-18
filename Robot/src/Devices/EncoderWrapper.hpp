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
        lastMillis = millis();
        count = encoder.read();
        lastCount = count;
        velocity = 0;
        lastVelocity = 0;
        acceleration = 0;
    }

    void flipDirection(){ isReversed = !isReversed;}

    void update()
    {
        unsigned long now = millis();
        count = encoder.read();
        
        unsigned long dt = now - lastMillis;
        if(dt != 0){
            long dCount = count - lastCount;
            velocity = (float)dCount * (1000.0 / dt);
            float dVelocity = velocity - lastVelocity;
            acceleration = dVelocity * (1000.0 / dt);
        }

        lastCount = count;
        lastVelocity = velocity;
        lastMillis = now;
    }

    int32_t read() { return isReversed? -count: count; }
    int32_t getCount() { return isReversed ? -count : count; }
    float getVelocity() { return isReversed ? -velocity : velocity; }
    float getAcceleration() { return isReversed ? -acceleration : acceleration; }

private:
    bool isReversed;
    uint8_t _pinA;
    uint8_t _pinB;
    Encoder encoder;
    int32_t count = 0;
    int32_t lastCount = 0;
    unsigned long lastMillis = 0;
    unsigned long INTERVAL = 100;
    float velocity = 0;
    float lastVelocity = 0;
    float acceleration = 0;
};
