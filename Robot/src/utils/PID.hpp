#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

struct PIDConstants
{
    float kp, ki, kd;
};

/**
 * Feedforward Fn relates a target value to a predicted signal. This is useful for linearizing control schemes where the environment is highly predictable. 
 */
typedef float (*FeedforwardFn)(float setpoint);

class PID
{
public:
    // Constructors (unchanged behavior)
    PID() : _kp(0.0), _ki(0.0), _kd(0.0) { _lastTime = millis();};

    PID(float kp,
        float ki,
        float kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    PID(PIDConstants consts)
        : _kp(consts.kp), _ki(consts.ki), _kd(consts.kd) {}

    /**
     * Get signal from controller
     *
     * @param measurement The current parameter being controller (can be position, velocity, etc)
     * @param dmeasurement The rate of change of the parameter of interest. If the PID is being used on position, this would be the velocity
     * @param setpoint The target value for the parameter of interest
     */
    virtual float update(float measurement, float dmeasurement, float setpoint)
    {
        unsigned long now = millis();
        unsigned long dt_ms = now - _lastTime;

        float dt = dt_ms * 0.001; // seconds

        float error = setpoint - measurement;

        // Integrator
        _integral += error * dt;

        _derivative = dmeasurement;

        float output =
            _kp * error + _kd * _derivative + _ki * _integral;

        // Save state
        _lastMeasurement = measurement;
        _lastTime = now;
        _lastOutput = output;

        return output;
    }


    /**
     * Sets all of measurements (error, integral, derivative) to zero
     */
    void reset()
    {
        _integral = 0.0;
        _derivative = 0.0;
        _lastMeasurement = 0.0;
        _lastOutput = 0.0;
        _lastTime = millis();
    }

    /**
     * Set the parameters of the PID controller
     */
    void set(float kp, float ki, float kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    /**
     * Set the parameters of the PID controller using a PIDConstants struct
     */
    void set(PIDConstants consts)
    {
        _kp = consts.kp;
        _ki = consts.ki;
        _kd = consts.kd;
    }

protected:
    float _kp{0.f}, _ki{0.f}, _kd{0.f};

    float _integral{0.f};
    float _derivative{0.f};
    float _lastMeasurement{0.f};
    float _lastOutput{0.f};
    unsigned long _lastTime{0};
};



class PIDController : public PID
{
public:
    // Constructors (unchanged behavior)
    PIDController(float kp, float ki, float kd)
        : PID(_kp, _ki, _kd), _kff(0.0f), _ffFunc(nullptr){}

    PIDController(PIDConstants consts)
        : PID(consts), _kff(0.0f), _ffFunc(nullptr) {}

    /**
     * Gets signal from controller. PIDConcontroller's update(...) differs from PID's update because of its implementation of a feedforward function. 
     *
     * @param measurement The current parameter being controller (can be position, velocity, etc)
     * @param dmeasurement The rate of change of the parameter of interest. If the PID is being used on position, this would be the velocity
     * @param setpoint The target value for the parameter of interest
     */
    float update(float measurement, float dmeasurement, float setpoint) override
    {
        unsigned long now = millis();
        unsigned long dt_ms = now - _lastTime;

        if (dt_ms == 0)
            return _lastOutput;

        float dt = dt_ms * 0.001; // seconds

        float error = setpoint - measurement;

        // Integrator
        _integral += error * dt;
        _integral = constrain(_integral, _iMin, _iMax); // anti-windup via clamping

        // Derivative (on measurement to avoid derivative kick from setpoint)

        // Feedforward: if user supplied a custom function use it; otherwise use simple kff * setpoint
        float feedforward = 0.0;
        if (_ffFunc)
        {
            feedforward = _ffFunc(setpoint);
        }
        else
        {
            feedforward = _kff * setpoint;
        }

        float output =
            _kp * error + _kd * dmeasurement + _ki * _integral + feedforward;

        // Save state
        _lastMeasurement = measurement;
        _lastTime = now;
        _lastOutput = output;

        return output;
    }

    /**
     * Update PID and return control output. This particular overload of update(...) finds the derivative of the current measurement. When derivative information is not available this can be used.
     * @param measurement The current parameter being controller (can be position, velocity, etc)
     * @param setpoint The target value for the parameter of interest
     */
    float update(float measurement, float setpoint) 
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


    /**
     * Sets the limits that the accumulated error term can reach. This helps to prevent integral windup
     */
    void setIntegralLimits(float minVal, float maxVal)
    {
        _iMin = minVal;
        _iMax = maxVal;
    }

    // -------- Feedforward related API --------

    /**
     * Set a simple feedforward gain: FF = kff * setpoint.
     * Default is 0 (no feedforward).
     */
    void setFeedforwardGain(float kff)
    {
        _kff = kff;
    }

    /**
     * Set a custom feedforward function.
     * The function signature must be:
     *   double f(float setpoint)
     *
     * If set, this function is used instead of the simple kff*setpoint. Pass nullptr to clear.
     */
    void setFeedforwardFunction(FeedforwardFn fn)
    {
        _ffFunc = fn;
    }

private:
    
    // Feedforward
    float _kff = 0.0;               // simple FF gain (FF = kff * setpoint)
    FeedforwardFn _ffFunc = nullptr; // optional custom FF function

    // State
    float _integral = 0.0;
    float _dFiltered = 0.0;
    float _lastMeasurement = 0.0;
    float _lastOutput = 0.0;

    // Settings
    float _iMin = -100000.0;
    float _iMax = 100000.0;
};

#endif