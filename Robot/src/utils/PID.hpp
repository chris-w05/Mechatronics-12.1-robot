#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

struct PIDConstants
{
    double kp, ki, kd;
};

typedef float (*FeedforwardFn)(float setpoint);

class PIDController
{
public:
    // Constructors (unchanged behavior)
    PIDController(double kp, double ki, double kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    PIDController(PIDConstants consts)
        : _kp(consts.kp), _ki(consts.ki), _kd(consts.kd) {}

    /**
     * Update PID and return control output.
     * Optionally provide setpointRate (setpoint derivative / commanded velocity) to be used by custom feedforward functions.
     */
    double update(double measurement, double setpoint, double setpointRate = 0.0)
    {
        unsigned long now = millis();
        unsigned long dt_ms = now - _lastTime;

        if (dt_ms == 0)
            return _lastOutput;

        double dt = dt_ms * 0.001; // seconds

        double error = setpoint - measurement;

        // Integrator
        _integral += error * dt;
        _integral = constrain(_integral, _iMin, _iMax); // anti-windup via clamping

        // Derivative (on measurement to avoid derivative kick from setpoint)
        double derivative = (measurement - _lastMeasurement) / dt;
        _dFiltered = _dAlpha * derivative + (1.0 - _dAlpha) * _dFiltered; // optional filter

        // Feedforward: if user supplied a custom function use it; otherwise use simple kff * setpoint
        double feedforward = 0.0;
        if (_ffFunc)
        {
            feedforward = _ffFunc(setpoint);
        }
        else
        {
            feedforward = _kff * setpoint;
        }

        double output =
            _kp * error + _kd * _dFiltered + _ki * _integral + feedforward;

        // Save state
        _lastMeasurement = measurement;
        _lastTime = now;
        _lastOutput = output;

        return output;
    }

    void reset()
    {
        _integral = 0.0;
        _dFiltered = 0.0;
        _lastMeasurement = 0.0;
        _lastOutput = 0.0;
        _lastTime = millis();
    }

    /***
     * 0 is the most aggressive filtering, 1 is the least aggressive
     */
    void setDerivativeFilter(double alpha)
    {
        _dAlpha = constrain(alpha, 0.0, 1.0);
    }

    void setIntegralLimits(double minVal, double maxVal)
    {
        _iMin = minVal;
        _iMax = maxVal;
    }

    void set(double kp, double ki, double kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    void set(PIDConstants consts)
    {
        _kp = consts.kp;
        _ki = consts.ki;
        _kd = consts.kd;
    }

    // -------- Feedforward related API --------

    /**
     * Set a simple feedforward gain: FF = kff * setpoint.
     * Default is 0 (no feedforward).
     */
    void setFeedforwardGain(double kff)
    {
        _kff = kff;
    }

    /**
     * Set a custom feedforward function.
     * The function signature must be:
     *   double f(double setpoint, double setpointRate, double measurement)
     *
     * If set, this function is used instead of the simple kff*setpoint. Pass nullptr to clear.
     */
    void setFeedforwardFunction(FeedforwardFn fn)
    {
        _ffFunc = fn;
    }

private:
    // PID gains
    double _kp = 0.0, _ki = 0.0, _kd = 0.0;

    // Feedforward
    double _kff = 0.0;               // simple FF gain (FF = kff * setpoint)
    FeedforwardFn _ffFunc = nullptr; // optional custom FF function

    // State
    double _integral = 0.0;
    double _dFiltered = 0.0;
    double _lastMeasurement = 0.0;
    double _lastOutput = 0.0;

    // Settings
    double _dAlpha = 1; // derivative smoothing
    double _iMin = -100000.0;
    double _iMax = 100000.0;

    unsigned long _lastTime = 0;
};

#endif