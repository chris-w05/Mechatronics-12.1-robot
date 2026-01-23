#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <Config.hpp>

class Encoder
{
public:

    Encoder(uint8_t pinA, uint8_t pinB)
        : _pinA(pinA), _pinB(pinB){}

    void init()
    {
        pinMode(_pinA, INPUT_PULLUP);
        pinMode(_pinB, INPUT_PULLUP);

        _lastA = digitalRead(_pinA);
        _lastTime = millis();
        _lastCount = _count;
    }

    void update()
    {
        bool A = digitalRead(_pinA);
        bool B = digitalRead(_pinB);

        if (A != _lastA)
        {
            _count += (A == B) ? 1 : -1;
            _lastA = A;
        }
    }

    long getCount() const
    {
        return _count;
    }


    /**
     * Returns position of encoder in revolutions
     */
    long getPosition() const
    {
        return _count/TICKS_PER_REV;
    }


    /**
     * Returns velocity of rotary encoder in revolutions per second
     */
    double getVelocity()
    {
        unsigned long now = millis();
        unsigned long dt_ms = now - _lastTime;

        if (dt_ms == 0)
            return 0.0;

        long count = _count;
        long dCount = count - _lastCount;

        _lastCount = count;
        _lastTime = now;

        double dt = dt_ms * 0.001;
        return dCount / (TICKS_PER_REV * dt); // ticks/sec
    }

    void reset()
    {
        _count = 0;
        _lastCount = 0;
        _lastTime = millis();
    }

    uint8_t getPinA() const { return _pinA; }
    uint8_t getPinB() const { return _pinB; }

private:
    uint8_t _pinA, _pinB;
    bool _lastA = false;

    volatile long _count = 0;

    long _lastCount = 0;
    unsigned long _lastTime = 0;
    int _ticks_per_unit = 1;
};

#endif // ENCODER_HPP