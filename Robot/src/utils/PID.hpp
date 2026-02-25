#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

/**
 * This file contains everything necessary to build a PID controller. It also has the added capability of being able to create a nonlinear controller using functions for kp ki and kd if wanted
 */


struct PIDConstants
{
    float kp, ki, kd;
};

/**
 * Feedforward Fn relates a target value to a predicted signal. This is useful for linearizing control schemes where the environment is highly predictable. 
 */
typedef float (*FeedforwardFn)(float setpoint);

/**
 * Kp function - creates a nonlinear kp term
 */
typedef float (*KpFn)(float measurement, float target);

/**
 * Kp function - creates a nonlinear ki term
 */
typedef float (*KiFn)(float measurement);

/**
 * Kp function - creates a nonlinear kd term
 */
typedef float (*KdFn)(float measurement);


/**
 * Nonlinear PID constant struct
 */
struct NonlinearPID{
    FeedforwardFn ff;
    KpFn kp;
    KiFn ki;
    KdFn kd;
};

class PID
{
public:
    // Constructors (unchanged behavior)
    PID() : _kp(0.0), _ki(0.0), _kd(0.0) { _lastTime = micros();};

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
        unsigned long now = micros();
        unsigned long dt_ms = now - _lastTime;

        float dt = dt_ms * 0.000001; // seconds

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
        _lastTime = micros();
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
     * Builds a nonlinear controller using constants and feedforward functions
     */
    PIDController(PIDConstants consts, NonlinearPID funcs)
        : PID(consts),
          _ffFunc(funcs.ff),
          _kpFunc(funcs.kp),
          _kiFunc(funcs.ki),
          _kdFunc(funcs.kd)
    {
    }

    /**
     * Builds a purely nonlinear controller
     */
    PIDController(NonlinearPID funcs)
        : PID(0, 0, 0),
        _ffFunc(funcs.ff), 
        _kpFunc(funcs.kp),
        _kiFunc(funcs.ki),
        _kdFunc(funcs.kd)
    {}

    /**
     * Gets signal from controller. PIDConcontroller's update(...) differs from PID's update because of its implementation of a feedforward function. 
     *
     * @param measurement The current parameter being controller (can be position, velocity, etc)
     * @param dmeasurement The rate of change of the parameter of interest. If the PID is being used on position, this would be the velocity
     * @param setpoint The target value for the parameter of interest
     */
    float update(float measurement, float dmeasurement, float setpoint) override
    {
        unsigned long now = micros();
        unsigned long dt_micros = now - _lastTime;

        if (dt_micros == 0)
            return _lastOutput;

        float dt = dt_micros * 0.000001; // seconds

        float error = setpoint - measurement;

        // Integrator
        _integral += error * dt;
        _integral = constrain(_integral, _iMin, _iMax); // anti-windup via clamping

        float output = calculate(measurement, setpoint, dmeasurement, _integral);

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
        unsigned long now = micros();
        unsigned long dt_micross = now - _lastTime;

        if (dt_micross == 0)
            return _lastOutput;

        float dt = dt_micross * 0.000001; // seconds

        float error = setpoint - measurement;

        // Integrator
        _integral += error * dt;
        _integral = constrain(_integral, _iMin, _iMax); // anti-windup via clamping

        // Derivative (on measurement to avoid derivative kick from setpoint)
        float derivative = (measurement - _lastMeasurement) / dt;
        derivative = (.1)*derivative + (.9)*_lastDerivative; //IIR filter with alpha = .1 


        // Feedforward: if user supplied a custom function use it; otherwise use simple kff * setpoint
        float output = calculate(measurement, setpoint, derivative, _integral );

        // Save state
        _lastMeasurement = measurement;
        _lastTime = now;
        _lastOutput = output;
        _lastDerivative = derivative;

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

    /**
     * Set a nonlinear function for Kp 
     */
    void setKpFunction(KpFn fn)
    {
        _kpFunc = fn;
    }

    /**
     * Set a nonlinear function for ki
     */
    void setKiFunction(KiFn fn)
    {
        _kiFunc = fn;
    }

    /**
     * Set a nonlinear function for kd
     */
    void setKdFunction(KdFn fn)
    {
        _kdFunc = fn;
    }

private:

    /**
     * Calculates the output of the controller
     * 
     * @param measurement Current measurement
     * @param setpoint The value that measurement is trying to achieve
     * @param derivative The time rate of change of measurement
     * @param integral The accumulated error of (Setpoint - measurement);
     */
    float calculate(float measurement, float setpoint, float derivative, float integral){
        float error = setpoint - measurement;
        float feedforward = 0.0;
        if (_ffFunc)
        {
            feedforward = _ffFunc(setpoint);
        }
        else
        {
            feedforward = _kff * setpoint;
        }

        float kpNonlinear = 0.0;
        if (_kpFunc)
        {
            kpNonlinear = _kpFunc(measurement, setpoint);
        }

        float kiNonlinear = 0.0;
        if (_kiFunc)
        {
            kiNonlinear = _kiFunc(_integral);
        }

        float kdNonlinear = 0.0;
        if (_kiFunc)
        {
            kdNonlinear = _kdFunc(derivative);
        }

        return _kp * error + _kd * derivative + _ki * integral + feedforward + kpNonlinear + kiNonlinear + kdNonlinear;
    }
    
    // Feedforward
    float _kff = 0.0;               // simple FF gain (FF = kff * setpoint)
    FeedforwardFn _ffFunc = nullptr; // optional custom FF function
    KpFn _kpFunc = nullptr;
    KiFn _kiFunc = nullptr;
    KdFn _kdFunc = nullptr;

    // State
    float _integral = 0.0;
    float _lastMeasurement = 0.0;
    float _lastOutput = 0.0;
    float _lastDerivative = 0.0;

    // Settings
    float _iMin = -100000.0;
    float _iMax = 100000.0;
};

#endif