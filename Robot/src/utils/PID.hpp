/**
 * @file PID.hpp
 * @brief PID controller classes with optional nonlinear extensions and feedforward support.
 *
 * Two classes are provided:
 * - `PID` — A lightweight base controller with proportional, integral, and derivative
 *   terms.  Derivative is supplied externally (from an encoder velocity signal, for example).
 * - `PIDController` — Extends PID with anti-windup, an IIR output filter, and optional
 *   nonlinear gain functions / feedforward callables for advanced control schemes.
 *
 * Function-pointer typedefs (#FeedforwardFn, #DerivativeFeedforwardFn, #KpFn, etc.) allow
 * the caller to inject arbitrary nonlinear behaviours without subclassing.
 */
#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <Arduino.h>

/**
 * @brief This file contains everything necessary to build a PID controller. It also has the added capability of being able to create a nonlinear controller using functions for kp ki and kd if wanted
 */


/**
 * @brief Proportional, integral, and derivative gains.
 *
 * Passed as a single struct to avoid long argument lists wherever gains need to
 * be configured or stored.
 */
struct PIDConstants
{
    float kp; ///< Proportional gain
    float ki; ///< Integral gain
    float kd; ///< Derivative gain
};

/**
 * @brief Feedforward function pointer type.
 *
 * Maps a setpoint value to a predicted steady-state motor signal.
 * Useful for linearising control where the plant response is predictable.
 * @param setpoint  The target value for the controlled variable.
 * @return          Estimated motor signal needed to achieve the setpoint.
 */
typedef float (*FeedforwardFn)(float setpoint);

/**
 * @brief Derivative feedforward function pointer type.
 *
 * Maps the rate of change of the setpoint to an additional feed-forward signal,
 * compensating for the lag introduced by a moving reference trajectory.
 * @param dsetpoint  Time derivative of the setpoint.
 * @return           Additional feedforward signal.
 */
typedef float (*DerivativeFeedforwardFn)(float dsetpoint);

/**
 * @brief Nonlinear proportional gain function pointer type.
 *
 * Replaces the constant kp term with a function of current measurement and target,
 * enabling gain-scheduling or error-dependent proportional control.
 * @param measurement  Current measured value.
 * @param target       Desired setpoint.
 * @return             Effective proportional control output.
 */
typedef float (*KpFn)(float measurement, float target);

/**
 * @brief Nonlinear integral gain function pointer type.
 *
 * Replaces the constant ki term with a function of the accumulated integral,
 * enabling conditional integration or anti-windup schemes.
 * @param measurement  Current measured value (or accumulated integral).
 * @return             Effective integral control output.
 */
typedef float (*KiFn)(float measurement);

/**
 * @brief Nonlinear derivative gain function pointer type.
 *
 * Replaces the constant kd term with a function of the current measurement
 * derivative, enabling derivative gain scheduling.
 * @param measurement  Current measurement (or its derivative).
 * @return             Effective derivative control output.
 */
typedef float (*KdFn)(float measurement);


/**
 * @brief Bundle of optional nonlinear gain functions for PIDController.
 *
 * Pass this to the #PIDController constructor to enable nonlinear behaviour.
 * Any member left as `nullptr` falls back to the corresponding linear gain.
 */
struct NonlinearPID{
    FeedforwardFn ff;           ///< Setpoint feedforward (replaces kff * setpoint)
    DerivativeFeedforwardFn dff; ///< Derivative feedforward (replaces kff * dSetpoint)
    KpFn kp;                    ///< Nonlinear proportional gain function
    KiFn ki;                    ///< Nonlinear integral gain function
    KdFn kd;                    ///< Nonlinear derivative gain function
};


/**
 * @brief Lightweight PID controller base class.
 *
 * The derivative term is supplied by the caller (e.g. from an encoder velocity
 * reading) rather than differentiating the measurement internally.  This avoids
 * noise amplification on slow-MCU platforms like the Arduino Mega.
 *
 * Use `PIDController` (the derived class) when you need anti-windup, output
 * filtering, or feedforward support.
 */
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
            _kp * error - _kd * _derivative + _ki * _integral;

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

    /**
     * Set the parameters of the PID controller using the value of the parameter and which one to set (0, 1, 2) for kp ki kd
     */
    void set(float value, short index)
    {
        switch(index){
            case 0:
                _kp = value;
                Serial.println(_kp);
                Serial.println("KP");
                break;
            case 1:
                _ki = value;
                Serial.println(_ki);
                Serial.println("KI");
                break;
            case 2:
                _kd = value;
                Serial.println(_kd);
                Serial.println("KD");
                break;
        }
    }

protected:
    float _kp{0.f}, _ki{0.f}, _kd{0.f};

    float _integral{0.f};
    float _derivative{0.f};
    float _lastMeasurement{0.f};
    float _lastOutput{0.f};
    unsigned long _lastTime{0};
};



/**
 * @brief Full-featured PID controller with anti-windup, output filtering, and optional nonlinear extensions.
 *
 * Extends #PID with:
 * - Integral clamping (anti-windup) via configurable limits.
 * - IIR output filter to smooth the command signal.
 * - Optional feedforward functions (#FeedforwardFn, #DerivativeFeedforwardFn).
 * - Optional nonlinear gain functions (#KpFn, #KiFn, #KdFn) for gain scheduling.
 *
 * When the nonlinear callbacks are `nullptr` the controller degrades to a
 * standard PID with the linear kp / ki / kd constants.
 */
class PIDController : public PID
{
public:
    PIDController() : PID(0,0,0), _kff(0.0f), _ffFunc(nullptr)
    {
    }

    // Constructors (unchanged behavior)
    PIDController(float kp, float ki, float kd)
        : PID(_kp, _ki, _kd), _kff(0.0f), _ffFunc(nullptr){
        updateIntegralLimitsFromKi(ki);
        }

    PIDController(PIDConstants consts)
        : PID(consts), _kff(0.0f), _ffFunc(nullptr) {
        updateIntegralLimitsFromKi(consts.ki);
        }

    
    /**
     * Builds a nonlinear controller using constants and feedforward functions
     */
    PIDController(PIDConstants consts, NonlinearPID funcs)
        : PID(consts),
          _ffFunc(funcs.ff),
          _dffFunc(funcs.dff),
          _kpFunc(funcs.kp),
          _kiFunc(funcs.ki),
          _kdFunc(funcs.kd)
    {
        updateIntegralLimitsFromKi(consts.ki);
    }

    /**
     * Builds a purely nonlinear controller
     */
    PIDController(NonlinearPID funcs)
        : PID(0, 0, 0),
        _ffFunc(funcs.ff), 
        _dffFunc(funcs.dff),
        _kpFunc(funcs.kp),
        _kiFunc(funcs.ki),
        _kdFunc(funcs.kd)
    {}

    /**
     * Set the values for all PID constants
     */
    void set(PIDConstants consts) { 
        _kp = consts.kp;
        _ki = consts.ki;
        _kd = consts.kd;
        updateIntegralLimitsFromKi(consts.ki);
    }

    /**
     * Set the parameters of the PID controller using the value of the parameter and which one to set (0, 1, 2) for kp ki kd
     */
    void set(float value, short index)
    {
        switch (index)
        {
        case 0:
            _kp = value;
            Serial.println(_kp);
            Serial.println("KP");
            break;
        case 1:
            _ki = value;
            updateIntegralLimitsFromKi(_ki);
            Serial.println(_ki);
            Serial.println("KI");
            break;
        case 2:
            _kd = value;
            Serial.println(_kd);
            Serial.println("KD");
            break;
        }
    }

    /**
     * Sets the strength of the internal IIR filter
     * 
     * @param alpha The strength of the filter, with 0 being the strongest and 1 being the weakest. Min of 0 and max of 1
     */
    void setFilterStrength( float alpha ){
        alpha = constrain( alpha, 0.0, 1.0);
        _alpha = alpha;
    }

    /**
     * Gets signal from controller. PIDConcontroller's update(...) differs from PID's update because of its implementation of a feedforward function. 
     *
     * @param measurement The current parameter being controller (can be position, velocity, etc)
     * @param dmeasurement The rate of change of the parameter of interest. If the PID is being used on position, this would be the velocity
     * @param setpoint The target value for the parameter of interest
     */
    float update(float measurement, float dmeasurement, float setpoint, float dSetpoint = 0)
    {
        unsigned long now = micros();
        unsigned long dt_micros = now - _lastTime;

        if (dt_micros == 0)
            return _lastOutput;

        float dt = dt_micros * 0.000001; // seconds

        float error = setpoint - measurement;

        // Integrator
        if (_ki != 0.0f || _kiFunc)
        {
            _integral += error * dt;
            _integral = constrain(_integral, _iMin, _iMax); // anti-windup via clamping
        }
        else
        {
            _integral = 0.0f;
        }

        float output = calculate(measurement, setpoint, dmeasurement, _integral, dSetpoint);

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
        if (_ki != 0.0f || _kiFunc)
        {
            _integral += error * dt;
            _integral = constrain(_integral, _iMin, _iMax); // anti-windup via clamping
        }
        else
        {
            _integral = 0.0f;
        }

        // Derivative (on measurement to avoid derivative kick from setpoint)
        float derivative = (measurement - _lastMeasurement) / dt;
        derivative = (.1)*derivative + (.9)*_lastDerivative; //IIR filter with alpha = .1 


        // Feedforward: if user supplied a custom function use it; otherwise use simple kff * setpoint
        float output = calculate(measurement, setpoint, derivative, _integral, 0);

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
     * Set a custon function in terms of the derivative of the target. 
     * the funciton signature needs to match:
     *  float function(float  arg)
     */
    void setDerivativeFeedforwardFunction(DerivativeFeedforwardFn fn)
    {
        _dffFunc = fn;
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

    /**
     * Gets rid of all of the nonlinear functions in the PID controller
     */
    void removeNonlinears(){
        _kpFunc = nullptr;
        _kiFunc = nullptr;
        _kdFunc = nullptr;
    }

private:
    void updateIntegralLimitsFromKi(float ki)
    {
        if (fabsf(ki) < 1e-6f)
        {
            _iMin = -100000.0f;
            _iMax = 100000.0f;
            _integral = 0.0f;
            return;
        }

        const float iLimitMagnitude = 400.0f / fabsf(ki);
        _iMin = -iLimitMagnitude;
        _iMax = iLimitMagnitude;
        _integral = constrain(_integral, _iMin, _iMax);
    }

    /**
     * Calculates the output of the controller
     * 
     * @param measurement Current measurement
     * @param setpoint The value that measurement is trying to achieve
     * @param derivative The time rate of change of measurement
     * @param integral The accumulated error of (Setpoint - measurement);
     */
    float calculate(float measurement, float setpoint, float derivative, float integral, float dSetpoint){
        float error = setpoint - measurement;
        float feedforward = 0.0;

        // nonlinear optional functions. In most cases these are zero. 
        if (_ffFunc)
        {
            feedforward = _ffFunc(setpoint);
        }
        else
        {
            feedforward = _kff * setpoint;
        }

        float dFeedForward = 0.0;
        if (_dffFunc){
            dFeedForward = _dffFunc(dSetpoint);
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
        float signal = _kp * error - _kd * (derivative - dSetpoint) + _ki * integral + 
            feedforward + dFeedForward + kpNonlinear + kiNonlinear + kdNonlinear;

        signal = (_alpha * signal) + (1-_alpha)*_lastOutput;
        _lastOutput = signal;
        return signal;

    }
    
    // Feedforward
    float _kff = 0.0;                        ///< Simple feedforward gain (FF = kff × setpoint)
    FeedforwardFn _ffFunc = nullptr;          ///< Custom setpoint feedforward function (overrides _kff when set)
    DerivativeFeedforwardFn _dffFunc = nullptr; ///< Custom derivative feedforward function
    KpFn _kpFunc = nullptr;                  ///< Nonlinear proportional gain function (overrides _kp when set)
    KiFn _kiFunc = nullptr;                  ///< Nonlinear integral gain function (overrides _ki when set)
    KdFn _kdFunc = nullptr;                  ///< Nonlinear derivative gain function (overrides _kd when set)

    float _alpha = 1;                        ///< IIR output filter coefficient (1 = off, smaller = stronger filter)

    // State
    float _integral = 0.0;                   ///< Accumulated integral term
    float _lastMeasurement = 0.0;            ///< Previous measurement (for internal derivative computation)
    float _lastOutput = 0.0;                 ///< Previous filtered output (for IIR filter and slew)
    float _lastDerivative = 0.0;             ///< Previous derivative estimate (for IIR derivative filtering)

    // Settings
    float _iMin = -100000.0;                 ///< Lower clamp on the integral accumulator (anti-windup)
    float _iMax = 100000.0;                  ///< Upper clamp on the integral accumulator (anti-windup)
};

#endif

