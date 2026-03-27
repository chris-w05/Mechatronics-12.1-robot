/**
 * @file DualMotorController.hpp
 * @brief Dual closed-loop motor controller wrapping an L298N driver with two independent PID loops.
 *
 * Each motor has its own #PIDController, slew-rate limiter, and optional feedforward /
 * nonlinear gain callbacks.  The class is designed for the drivetrain (left + right wheels)
 * but is also used for the shooter's single-motor controller via one of the single-motor
 * `update()` / `setPower()` overloads.
 */
#ifndef DUAL_MOTOR_CONTROLLER_HPP
#define DUAL_MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "utils/PID.hpp"
#include <DualTB9051FTGMotorShield.h>
#include <L298NMotorDriverMega.h>


/**
 * @brief Pin bundle for a dual-channel L298N-style motor driver.
 *
 * Holds the PWM and direction pin numbers for both motor channels.  Pass an
 * instance to `DualMotorController`'s constructor.
 */
struct TB9051Pins
{
    uint8_t m1PWM, m1Direction1, m1Direction2; ///< Motor 1 (left / primary) control pins
    uint8_t m2PWM, m2Direction1, m2Direction2; ///< Motor 2 (right / secondary) control pins
};

/**
 * @brief Dual-channel closed-loop motor controller.
 *
 * Wraps two independent #PIDController instances driving an L298N motor
 * shield.  Supports position and velocity control modes, optional motor
 * direction reversal, a configurable slew-rate limiter, and a suite of
 * feedforward / nonlinear gain injection helpers.
 */
class DualMotorController
{
public:
    // Remapped-pins constructor (shield #2, #3, ...)
    DualMotorController(
        const TB9051Pins &pins,
        PIDConstants M1PIDConsts = { 0, 0, 0}, bool m1reversed = false,
        PIDConstants M2PIDConsts = {0, 0, 0}, bool m2reversed = false,
        bool holdPositionWhenStopped1 = false,
        bool holdPositionWhenStopped2 = false)
        : // dualDriver(
          //       pins.m1en, pins.m1dir, pins.m1pwm, pins.m1diag, pins.m1ocm,
          //       pins.m2en, pins.m2dir, pins.m2pwm, pins.m2diag, pins.m2ocm),'
          dualDriver(
            pins.m1PWM, pins.m1Direction1, pins.m1Direction2, 
            pins.m2PWM, pins.m2Direction1, pins.m2Direction2),
          _m1reversed(m1reversed),
          _m2reversed(m2reversed),
          _pid1(M1PIDConsts),
          _pid2(M2PIDConsts),
          _holdPositionWhenStopped1(holdPositionWhenStopped1),
          _holdPositionWhenStopped2(holdPositionWhenStopped2)
    {
    }

    enum ControlMode{
        POSITION,
        VELOCITY
    };


    /**
     * Initialize the controller.
     */
    void init()
    {
        dualDriver.init();
        Serial.println("Drivetrain Drivers enabled");
        dualDriver.flipM1(_m1reversed);
        dualDriver.flipM2(_m2reversed);
    }


    /**
     * Set a target value for the motor controller's PID loop. This is unitless as units are handled by the implementation of the PID controller
     *
     * @param targetM1 Desired value for M1
     * @param targetM2 Desired value for M2, defaults to 0.0
     **/
    void setTarget(double targetM1, double targetM2 = 0.0)
    {
        _target1 = targetM1;
        _target2 = targetM2;
    }

    /**
     * Should be called when There is a large jump in target values, I.E. going from full forward velocity to full reverse
     */
    void resetPID(){
        _pid1.reset();
        _pid2.reset();
    }

    void makeLinear(){
        _pid1.removeNonlinears();
        _pid2.removeNonlinears();
    }


    /**
     * Set a commanded power to both motors
     *
     * @param signalM1 Clamped to -400, 400
     * @param signalM2 Clamped to -400, 400
     */
    void setPower( int signalM1, int signalM2 ){
        dualDriver.setM1Speed(signalM1);
        dualDriver.setM2Speed(signalM2);
    }

    /** 
     * Set a commanded power to a specific motor
     * 
     * @param signal Power to send to motor, from -400 to 400
     * @param motor 0 -> M1,  1 -> M2
     */
    void setPower(int signal, bool motor = 0)
    {
        if (!motor){
            dualDriver.setM1Speed(signal);
            return;
        }
        dualDriver.setM2Speed(signal);
    }

    /**
     * Update the PID control loop for motor controller
     *
     * @param current_value1 The current state of motor 1 (position/velocity/etc.)
     * @param current_value1 The current time derivative of motor 1 (position/velocity/etc.)
     * @param current_value2 The current state of motor 2 (position/velocity/etc.) This defaults to 0.0 for use when manipulating only 1 motor
     * @param current_value2 The current time derivative of motor 2 (position/velocity/etc.)
     * @param dSetPoint1     The rate of change of the target value for 1
     * @param dSetPoint2     The rate of change of the target value for 2
     */
    void update(
        float current_value1, float dcurrent_value1,
        float current_value2, float dcurrent_value2, 
        float dSetPoint1 = 0, float dSetPoint2 = 0)
    {
        int pidOut1 = _pid1.update(current_value1, dcurrent_value1, _target1, dSetPoint1);
        int pidOut2 = _pid2.update(current_value2, dcurrent_value2, _target2, dSetPoint2);

        // Compute deltas relative to last sent signals
        int delta1 = pidOut1 - lastSignal1;
        int delta2 = pidOut2 - lastSignal2;

        /**Slew limiter - controlls acceleration of motors - if this is not necessary, set the slew rate to 800 (full range of possible command values)
         * */
        if (delta1 > slewRateLimiter)
            pidOut1 = lastSignal1 + slewRateLimiter;
        else if (delta1 < -slewRateLimiter)
            pidOut1 = lastSignal1 - slewRateLimiter;

        if (delta2 > slewRateLimiter)
            pidOut2 = lastSignal2 + slewRateLimiter;
        else if (delta2 < -slewRateLimiter)
            pidOut2 = lastSignal2 - slewRateLimiter;

        // Debug prints: show PID output, limited send value, and the delta from last sent value
        // Serial.print("V L: ");
        // Serial.print(current_value1);
        // Serial.print("  tL: ");
        // Serial.print(_target1);
        // Serial.print("  SglL ");
        // Serial.print(pidOut1);

        // Serial.print("\t\tV R: ");
        // Serial.print(current_value2);
        // Serial.print("  tR: ");
        // Serial.print(_target2);
        // Serial.print("  SglR ");
        // Serial.println(pidOut2);

        // Send the limited signals to the driver
        // Serial.print(">TargetL:");
        // Serial.print(_target1);
        // Serial.print(",Measurement1L:");
        // Serial.print(current_value1);
        // Serial.print(",PidL:");
        // Serial.print(pidOut1);
        // Serial.print(",TargetR:");
        // Serial.print(_target2);
        // Serial.print(",MeasurementR:");
        // Serial.print(current_value2);
        // Serial.print(",PidR:");
        // Serial.print(pidOut2);
        // Serial.println("\r");

        dualDriver.setM1Speed(pidOut1);
        dualDriver.setM2Speed(pidOut2);

        // Save last sent values for next iteration
        lastSignal1 = pidOut1;
        lastSignal2 = pidOut2;
    }

    /**
     * Update the PID control loop for motor controller
     *
     * @param current_value The current state of motor 1 (position/velocity/etc.)
     * @param current_value The current time derivative of motor 1 (position/velocity/etc.)
     * @param motor Which motor to update the PID control for. 0 for M1, 1 for M2
     */
    void update(
        float current_value, float dcurrent_value, bool motor = 0)
    {

        PIDController &pid = motor == 0 ? _pid1 : _pid2;
        float &target = motor ==0 ? _target1 : _target2;
        int &lastSignal = motor == 0? lastSignal1 : lastSignal2;

        float pidOut = pid.update(current_value, dcurrent_value, target);

        // Compute deltas relative to last sent signals
        float delta = pidOut - lastSignal;
        /**Slew limiter - controlls acceleration of motors - if this is not necessary, set the slew rate to 800 (full range of possible command values)
         * */
        if (delta > slewRateLimiter)
            pidOut = lastSignal + slewRateLimiter;

        // Debug prints: show PID output, limited send value, and the delta from last sent value
        // Serial.print("Current value: ");
        // Serial.print(current_value);
        // Serial.print("  target: ");
        // Serial.print(_target1);
        // Serial.print("  Signal: ");
        // Serial.println(pidOut);

        // Send the limited signals to the driver
        motor == 0 ? dualDriver.setM1Speed(pidOut) : dualDriver.setM2Speed(pidOut);

        lastSignal = pidOut;
    }

    /**
     * Use this when a derivative for both motors is not available
     * The arguments do not include the derivatives of the state of the system
     * 
     * @param current_value1 The current measurement of system controlled by M1
     * @param current_value2 The current measurement of system controlled by M2
     */
    void updateWithoutDerivatice(float current_value1, float current_value2){
        int pidOut1 = _pid1.update(current_value1, _target1);
        int pidOut2 = _pid2.update(current_value2, _target2);

        // Compute deltas relative to last sent signals
        int delta1 = pidOut1 - lastSignal1;
        int delta2 = pidOut2 - lastSignal2;

        /**Slew limiter - controlls acceleration of motors - if this is not necessary, set the slew rate to 800 (full range of possible command values)
         * */
        if (delta1 > slewRateLimiter)
            pidOut1 = lastSignal1 + slewRateLimiter;
        else if (delta1 < -slewRateLimiter)
            pidOut1 = lastSignal1 - slewRateLimiter;

        if (delta2 > slewRateLimiter)
            pidOut1 = lastSignal2 + slewRateLimiter;
        else if (delta2 < -slewRateLimiter)
            pidOut2 = lastSignal2 - slewRateLimiter;

        unsigned long now = millis();
        if(now - lastMillis > 100 ){
            // Serial.print(">VL:");
            // Serial.print(current_value1);
            // // Serial.print(",tL:");
            // // Serial.print(_target1);
            // Serial.print(",SglL:");
            // Serial.println();
            // // Serial.print(pidOut1);
            // // Serial.print(",VR:");
            // // Serial.print(current_value2);
            // // Serial.print(",tR:");
            // // Serial.print(_target2);
            // // Serial.print(",SglR:");
            // Serial.println(pidOut2);
            lastMillis = now;
        }


        // Send the limited signals to the driver
        dualDriver.setM1Speed(pidOut1);
        dualDriver.setM2Speed(pidOut2);

        // Save last sent values for next iteration
        lastSignal1 = pidOut1;
        lastSignal2 = pidOut2;
    }


    /**
     * Stop the motors.
     */
    void stop()
    {
        if (!_holdPositionWhenStopped1)
        {
            _target1 = 0.0;
            _pid1.reset();
            dualDriver.setM1Speed(0);
        }
        if (!_holdPositionWhenStopped2)
        {
            _target2 = 0.0;
            _pid2.reset();
            dualDriver.setM2Speed(0);
        }
    }


    /**
     * Set the constants of the PID controller for one motor to a stuct of constants
     * 
     * @param consts The values to set kp, ki, and kd to
     * @param motor Which motor to apply constant to, 0 for M1, 1 for M2
     */
    void setPID( PIDConstants consts, bool motor = 0){
        motor == 0 ? _pid1.reset() : _pid2.reset();
        motor == 0 ? _pid1.set(consts) : _pid2.set(consts);};

    /**
     * Set the constants of both PID controllers
     *
     * @param consts The values to set kp, ki, and kd for M1
     * @param consts2 The values to set kp, ki, and kd for M2
     */
    void setPID(PIDConstants consts, PIDConstants consts2)
    {
        _pid1.reset();
        _pid2.reset();
        _pid1.set(consts);
        _pid2.set(consts2);
    };


    /**
     * Sets a PID constant (index) to a specificed value (value)
     */
    void setPIDConstant(float value, short index)
    {
        _pid1.set(value, index);
        _pid2.set(value, index);
        _pid1.reset();
        _pid2.reset();
    };

    /**
     * Set the feedforward function for one motor's controller
     * 
     * @param fcn The function to dictate motor bias. 
     *  Inputs: target measurement
     *  Outputs: motor signal
     * @param motor Which motor to apply function to. 0 for M1, 1 for M2
     */
    void setPIDFeedForwardFunc(FeedforwardFn fcn, bool motor = 0){
        motor == 0 ? _pid1.reset() : _pid2.reset();
        motor == 0 ? _pid1.setFeedforwardFunction(fcn) : _pid2.setFeedforwardFunction(fcn);
    }


    /**
     * Set the feedforward function for one motor's controller
     *
     * @param fcn The function to dictate motor bias.
     *  Inputs: target measurement
     *  Outputs: motor signal
     * 
     * @param fcn2 The function to dictate motor bias.
     *  Inputs: target measurement
     *  Outputs: motor signal
     * */
    void setPIDFeedForwardFunc(FeedforwardFn fcn, FeedforwardFn fcn2)
    {
        _pid1.reset();
        _pid2.reset();
        _pid1.setFeedforwardFunction(fcn);
        _pid2.setFeedforwardFunction(fcn2);
    }

    /**
     * Set the derivative feedforward function for one motor's controller. This function handles the case where the target position changes with time so an expected signal can be added in terms of the rate of change of the target position. 
     *
     * @param fcn The function to dictate motor power by target rate.
     *  Inputs: setpoint time derivative
     *  Outputs: motor signal
     * @param motor Which motor to apply function to. 0 for M1, 1 for M2
     */
    void setPIDDerivativeFeedForwardFunc(DerivativeFeedforwardFn fcn, bool motor = 0)
    {
        motor == 0 ? _pid1.reset() : _pid2.reset();
        motor == 0 ? _pid1.setDerivativeFeedforwardFunction(fcn) : _pid2.setDerivativeFeedforwardFunction(fcn);
    }

    /**
     * Set the derivative feedforward function for both motors' controllers. This function handles the case where the target position changes with time so an expected signal can be added in terms of the rate of change of the target position.
     *
     * @param fcn The function to dictate motor power by target rate.
     *  Inputs: setpoint time derivative
     *  Outputs: motor signal
     * @param fcn2 The function to dictate motor power by target rate.
     *  Inputs: setpoint time derivative
     *  Outputs: motor signal
     */
    void setPIDDerivativeFeedForwardFunc(DerivativeFeedforwardFn fcn, DerivativeFeedforwardFn fcn2)
    {
        _pid1.reset();
        _pid2.reset();
        _pid1.setDerivativeFeedforwardFunction(fcn);
        _pid2.setDerivativeFeedforwardFunction(fcn2);
    }

    /**
     * Set the nonlinear kp function for a specific motor
     *
     * @param fcn The nonlinear kP function
     *  Inputs: current measurement, target measurement
     *  Outputs: motor signal
     * @param motor Which motor to apply function to. 0 for M1, 1 for M2
     */
    void setPIDKpFunction(KpFn fcn, bool motor = 0){
        motor == 0 ? _pid1.reset() : _pid2.reset();
        motor == 0 ? _pid1.setKpFunction(fcn) : _pid2.setKpFunction(fcn);
    }

    /**
     * Set the nonlinear kp function for two both motor controllers
     *
     * @param fcn The function to dictate motor bias.
     *  Inputs: current measurement, target measurement
     *  Outputs: motor signal
     *
     * @param fcn2 The function to dictate motor bias.
     *  Inputs: current measurement, target measurement
     *  Outputs: motor signal
     * */
    void setPIDKpFunction(KpFn fcn, KpFn fcn2)
    {
        _pid1.reset();
        _pid2.reset();
        _pid1.setKpFunction(fcn);
        _pid2.setKpFunction(fcn2);
    }

    /**
     * Set a particular motor's ki to a nonlinear function
     * */
    void setPIDkiFunction(KiFn fcn, bool motor = 0){
        motor == 0 ? _pid1.reset() : _pid2.reset();
        motor == 0 ? _pid1.setKiFunction(fcn) : _pid2.setKiFunction(fcn);
    }

    /**
     * Set both motors' ki to nonlinear functions
     */
    void setPIDKpFunction(KiFn fcn, KiFn fcn2)
    {
        _pid1.reset();
        _pid2.reset();
        _pid1.setKiFunction(fcn);
        _pid2.setKiFunction(fcn2);
    }

    /**
     * Set a particular motor's kd to a nonlinear function
     * */
    void setPIDkdFunction(KdFn fcn, bool motor = 0)
    {
        motor == 0 ? _pid1.reset() : _pid2.reset();
        motor == 0 ? _pid1.setKdFunction(fcn) : _pid2.setKdFunction(fcn);
    }

    /**
     * Set both motors' kd to nonlinear functions
     */
    void setPIDKdFunction(KdFn fcn, KdFn fcn2)
    {
        _pid1.reset();
        _pid2.reset();
        _pid1.setKdFunction(fcn);
        _pid2.setKdFunction(fcn2);
    }

    

    // float getM1current() { return dualDriver.getM1CurrentMilliamps(); }
    // float getM2current() { return dualDriver.getM2CurrentMilliamps(); }

private:
    // DualTB9051FTGMotorShield dualDriver;
    L298NMotorDriverMega dualDriver;

    bool _m1reversed = false;
    bool _m2reversed = true;

    unsigned long lastMillis = 0;

    PIDController _pid1;
    PIDController _pid2;

    bool _holdPositionWhenStopped1 = false;
    bool _holdPositionWhenStopped2 = false;

    int lastSignal1 = 0;
    int lastSignal2 = 0;

    float _target1 = 0.0;
    float _target2 = 0.0;

    /**
     * Maximum change in motor power per cycle
     */
    int slewRateLimiter = 800;

    
};

#endif
