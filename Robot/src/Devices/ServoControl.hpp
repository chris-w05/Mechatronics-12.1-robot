/**
 * @file ServoControl.hpp
 * @brief Buffered servo wrapper that defers writes until `update()` is called.
 *
 * Owned by the Miner and Ramp subsystems.  The angle is clamped to a
 * configurable [minAngle, maxAngle] range and optionally mirrored so that
 * the caller can think in terms of "logical" angles independent of the
 * physical mounting orientation.
 */
#ifndef SERVO_SUBSYSTEM_H
#define SERVO_SUBSYSTEM_H

#include <Arduino.h>
#include <PWMServo.h>

/**
 * @brief Servo driver with angle clamping, inversion, and deferred writes.
 *
 * Set a target angle with `setAngle()`, then call `update()` to send it
 * to the servo hardware.  Separating set from send lets multiple setAngle
 * calls within one loop iteration reduce to a single PWM update.
 */
class ServoControl
{
public:
    /**
     * @brief Construct the servo wrapper.
     * @param pin       Arduino PWM pin for the servo signal.
     * @param minAngle  Minimum allowed servo angle (degrees, default 0).
     * @param maxAngle  Maximum allowed servo angle (degrees, default 180).
     * @param inverted  If true, commands are mirrored around the midpoint.
     */
    ServoControl(uint8_t pin,
                   int minAngle = 0,
                   int maxAngle = 180,
                   bool inverted = false):
                _pin(pin),
                _minAngle(minAngle),
                _maxAngle(maxAngle),
                _targetAngle(minAngle),
                _currentAngle(minAngle),
                _inverted(inverted)
                {}

    /** @brief Attach the servo to its pin and write the initial position. */
    void init(){
        _servo.attach(_pin);
        _attached = _servo.attached();
        _currentAngle = _targetAngle;

        if (_attached)
        {
            _servo.write(_currentAngle);
        }
        else
        {
            Serial.print("Servo attach failed on pin ");
            Serial.println(_pin);
        }
    }

    /** @brief Write the target angle to the servo if it has changed. Must be called every loop. */
    void update(){
        if (!_enabled || !_attached)
            return;

        if (_currentAngle != _targetAngle)
        {
            _currentAngle = _targetAngle;
            _servo.write(_currentAngle);
        }
    }

    /**
     * @brief Set the desired servo angle.
     *
     * The angle is clamped to [minAngle, maxAngle] before storage.  If
     * `inverted` was set at construction, the angle is mirrored.
     * @param angle  Target angle in degrees.
     */
    void setAngle(int angle){
        angle = clamp(angle);

        if (_inverted)
        {
            angle = _maxAngle - (angle - _minAngle);
        }

        _targetAngle = angle;
    }

    /** @return The most recently written servo angle (degrees). */
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
    PWMServo _servo;    ///< Underlying PWMServo library object
    uint8_t _pin;       ///< Arduino pin number for servo signal

    int _minAngle;      ///< Lower bound for angle clamping
    int _maxAngle;      ///< Upper bound for angle clamping
    int _targetAngle;   ///< Angle requested by setAngle(), pending update()
    int _currentAngle;  ///< Angle last written to the servo

    bool _enabled  = true;  ///< When false, update() is skipped
    bool _attached = false; ///< True after a successful servo.attach()
    bool _inverted;         ///< Mirror angle around the midpoint when true

    /** @brief Clamp angle to [_minAngle, _maxAngle]. */
    int clamp(int angle) const{
        if (angle < _minAngle)
            return _minAngle;
        if (angle > _maxAngle)
            return _maxAngle;
        return angle;
    }
};

#endif
