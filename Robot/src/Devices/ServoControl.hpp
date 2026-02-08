#ifndef SERVO_SUBSYSTEM_H
#define SERVO_SUBSYSTEM_H

#include <Arduino.h>
#include <PWMServo.h>

class ServoControl
{
public:
    ServoControl(uint8_t pin,
                   int minAngle = 0,
                   int maxAngle = 180,
                   bool inverted = false);

    void init();
    void update();

    void setAngle(int angle);
    int getAngle() const;

    void enable();
    void disable();
    bool isEnabled() const;

private:
    PWMServo _servo;
    uint8_t _pin;

    int _minAngle;
    int _maxAngle;
    int _targetAngle;
    int _currentAngle;

    bool _enabled;
    bool _inverted;

    int clamp(int angle) const;
};

#endif
