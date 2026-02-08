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
        :  encoder(pinA, pinB)
    {
    }

    // Call in setup()
    void init()
    {
        lastMillis = millis();
        count = 0;
        velocity = 0;
        lastCount = 0;
    }

    // Call in your loop at least as fast as the sensor's update rate (~60 Hz typical)
    void update()
    {
        unsigned long now = millis();
        if (now - lastMillis >= INTERVAL)
        {
            count = encoder.read();

            long dCount = count - lastCount;
            // velocity = counts per second
            velocity = (float)dCount * (1000.0 / INTERVAL);
            float dVelocity = velocity - lastVelocity;
            acceleration = dVelocity * (1000.0 / INTERVAL);

            lastCount = count;
            lastVelocity = velocity;
            lastMillis = now;
        }
    }

    int32_t read(){
        return count;
    }

    int32_t getCount(){
        return count;
    }

    float getVelocity(){
        return velocity;
    }

    float getAcceleration(){
        return acceleration;
    }

private:
    uint8_t _pinA;
    uint8_t _pinB;
    Encoder encoder;
    int32_t count;
    int32_t lastCount;

    unsigned long lastMillis;
    unsigned long INTERVAL = 100; //ms - update time to take average velocity

    float velocity;
    float lastVelocity;
    float acceleration;
};