#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : _kp(kp), _ki(ki), _kd(kd){}

    double update(double measurement, double setpoint)
    {
        unsigned long now = millis();
        unsigned long dt_ms = now - _lastTime;

        if (dt_ms == 0)
            return _lastOutput;

        double dt = dt_ms * 0.001; // seconds

        double error = setpoint - measurement;

        
        _integral += error * dt;
        _integral = constrain(_integral, _iMin, _iMax); //Prevents error from building up if things like PWM get maxxed out


        double derivative = (measurement - _lastMeasurement) / dt;
        _dFiltered = derivative;
        // _dFiltered = _dAlpha * derivative + (1.0 - _dAlpha) * _dFiltered; //Filters noise from velocity readings, may or may not be needed

        double output =
            _kp * error + _kd * _dFiltered + _ki * _integral;

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

    void set( double kp, double ki, double kd){
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

private:
    double _kp, _ki, _kd;

    double _integral = 0.0;
    double _dFiltered = 0.0;
    double _lastMeasurement = 0.0;
    double _lastOutput = 0.0;

    double _dAlpha = 0.1; // derivative smoothing

    double _iMin = -100000.0;
    double _iMax = 100000.0;

    unsigned long _lastTime = 0;
};

#endif