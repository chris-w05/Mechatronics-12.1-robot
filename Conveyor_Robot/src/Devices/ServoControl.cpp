#include "Devices/ServoControl.hpp"

ServoControl::ServoControl(uint8_t pin,
                               int minAngle,
                               int maxAngle,
                               bool inverted)
    : _pin(pin),
      _minAngle(minAngle),
      _maxAngle(maxAngle),
      _targetAngle(minAngle),
      _currentAngle(minAngle),
      _enabled(false),
      _inverted(inverted) {}

void ServoControl::init()
{
    _servo.attach(_pin);
    _servo.write(_currentAngle);
}

void ServoControl::update()
{
    if (!_enabled)
        return;

    if (_currentAngle != _targetAngle)
    {
        _currentAngle = _targetAngle;
        _servo.write(_currentAngle);
    }
}

void ServoControl::setAngle(int angle)
{
    angle = clamp(angle);

    if (_inverted)
    {
        angle = _maxAngle - (angle - _minAngle);
    }

    _targetAngle = angle;
}

int ServoControl::getAngle() const
{
    return _currentAngle;
}

void ServoControl::enable()
{
    _enabled = true;
}

void ServoControl::disable()
{
    _enabled = false;
}

bool ServoControl::isEnabled() const
{
    return _enabled;
}

int ServoControl::clamp(int angle) const
{
    if (angle < _minAngle)
        return _minAngle;
    if (angle > _maxAngle)
        return _maxAngle;
    return angle;
}
