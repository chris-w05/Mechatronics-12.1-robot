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
                   bool inverted = false):
                _pin(pin),
                _minAngle(minAngle),
                _maxAngle(maxAngle),
                _inverted(inverted)
                {}

    void init(){
        _servo.attach(_pin);
        _servo.write(_currentAngle);
    }

    void update(){
        if (!_enabled)
            return;

        if (_currentAngle != _targetAngle)
        {
            _currentAngle = _targetAngle;
            _servo.write(_currentAngle);
        }
    }

    void setAngle(int angle){
        angle = clamp(angle);

        if (_inverted)
        {
            angle = _maxAngle - (angle - _minAngle);
        }

        _targetAngle = angle;
    }

    int getAngle() const{
        return _currentAngle;
    }

    void enable(){
        _enabled = true;
    }

    void disable(){
        _enabled = false;
    }

    bool isEnabled() const{
        return _enabled;
    }

private:
    PWMServo _servo;
    uint8_t _pin;

    int _minAngle;
    int _maxAngle;
    int _targetAngle;
    int _currentAngle;

    bool _enabled = true;
    bool _inverted;

    int clamp(int angle) const{
        if (angle < _minAngle)
            return _minAngle;
        if (angle > _maxAngle)
            return _maxAngle;
        return angle;
    }
};

#endif
