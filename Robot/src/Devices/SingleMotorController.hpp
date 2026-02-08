#pragma once
#ifndef SINGLE_MOTOR_CONTROLLER_HPP
#define SINGLE_MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "L298NMotorDriverMega.h"
#include "utils/PID.hpp"

/*
  SingleMotorController.hpp

  Thin, Arduino-friendly wrapper around L298NMotorDriverMega to control a single motor (M1).
  Mirrors the style / API shape used in your DualMotorController but only for one motor.

  Usage notes:
  - You can construct this with an existing L298NMotorDriverMega instance (preferred if you manage both motors),
    or provide the 6 pins used to construct an internal L298NMotorDriverMega instance.
  - This wrapper controls only M1 (calls setM1Speed / flipM1 / setM1Brake). If you want to use M2,
    create another wrapper or use the L298NMotorDriverMega instance directly.
  - PIDController is optional — the controller's update() method will use it if enabled.
*/

class SingleMotorController
{
public:
    // Control mode for the controller
    enum ControlMode {
        POSITION,
        VELOCITY,
        OPEN_LOOP  // direct power/speed control with no PID
    };

    // -----------------------
    // Constructors
    // -----------------------


    // 2) Create and own an internal L298NMotorDriverMega using explicit pins.
    //    You must supply all six pins (M1EN, M1C, M1D, M2EN, M2C, M2D) because that's the L298 constructor signature.
    /* IMPORTANT - M2 Pins are not needed, but they cannot interfere with other functions on board*/
    SingleMotorController(uint8_t M1EN,
                          uint8_t M1C,
                          uint8_t M1D,
                          uint8_t M2EN,
                          uint8_t M2C,
                          uint8_t M2D,
                          bool m1reversed = false,
                          ControlMode mode = OPEN_LOOP,
                          double kp = 0.0, double ki = 0.0, double kd = 0.0,
                          bool holdPositionWhenStopped = false)
        : _internalDriver(M1EN, M1C, M1D, M2EN, M2C, M2D),
          _m1reversed(m1reversed),
          _controlMode(mode),
          _pid(kp, ki, kd),
          _holdPositionWhenStopped(holdPositionWhenStopped)
    {
    }

    // Destructor: delete internal driver if we created it
    ~SingleMotorController()
    {
        _internalDriver;
    }

    // -----------------------
    // Initialization
    // -----------------------

    // Initialize underlying driver and set direction flip if required.
    void init()
    {
        L298NMotorDriverMega *drv = getDriver();
        if (!drv) return;
        drv->init();
        // flipM1 expects a boolean indicating whether to invert directions
        drv->flipM1(_m1reversed);
    }

    // -----------------------
    // Configuration
    // -----------------------

    // Set control mode (OPEN_LOOP / VELOCITY / POSITION)
    void setControlMode(ControlMode mode) { _controlMode = mode; }

    // Update PID gains at runtime
    void setPIDGains(double kp, double ki, double kd) { _pid.set(kp, ki, kd); }

    // Manually flip M1 direction
    void flipM1(bool flip) {
        _m1reversed = flip;
        L298NMotorDriverMega *drv = getDriver();
        if (drv) drv->flipM1(flip);
    }

    // -----------------------
    // High-level API (like DualMotorController)
    // -----------------------

    // Set the target used by the PID (units depend on your sensor & mode)
    void setTarget(double target) { _target = target; }

    // Directly set power/signal to the motor (bypasses PID)
    // Accepts typical L298 range: -400..400 (matches DualMotorController's conventions)
    void setPower(int signal) {
        L298NMotorDriverMega *drv = getDriver();
        if (!drv) return;
        drv->setM1Speed(signal);
    }

    // Call regularly to run PID loop (if not OPEN_LOOP)
    // current_value should be a sensor reading appropriate to the control mode (e.g., velocity or position)
    void update(double current_value)
    {
        L298NMotorDriverMega *drv = getDriver();
        if (!drv) return;

        if (_controlMode == OPEN_LOOP) {
            // nothing to do for PID — user uses setPower()
            return;
        }

        double signal = _pid.update(current_value, _target);
        // keep signal within the same bounds used in your DualMotorController
        signal = constrain(signal, -400.0, 400.0);
        drv->setM1Speed((int)signal);
    }

    // Stop the motor. If holdPositionWhenStopped is true, PID target is preserved,
    // otherwise PID state/target are reset and motor power set to 0.
    void stop()
    {
        L298NMotorDriverMega *drv = getDriver();
        if (!drv) return;

        if (!_holdPositionWhenStopped)
        {
            _target = 0.0;
            _pid.reset();
            drv->setM1Speed(0);
        }
        else
        {
            // If holding, keep target but zero the output only if you want a brake.
            // Here we'll actively brake to hold position if in POSITION mode; otherwise leave it
            if (_controlMode == POSITION) {
                // keep PID running; do not zero the controller
                // the user should call update() frequently so PID can act to hold
            } else {
                drv->setM1Speed(0);
            }
        }
    }

    // Set motor brake (0..400 typical)
    void setBrake(int brake) {
        L298NMotorDriverMega *drv = getDriver();
        if (!drv) return;
        drv->setM1Brake(brake);
    }

    // -----------------------
    // Diagnostics / helpers
    // -----------------------

    // NOTE: L298NMotorDriverMega may not provide current-sensing methods.
    // If you have a current-sense circuit, call your sensor reading code instead.
    // This function returns 0.0 by default; override/extend if you add current sensing.
    float getMotorCurrentMilliamps() {
        // Placeholder: the dual wrapper used getM1CurrentMilliamps(), but
        // L298NMotorDriverMega doesn't expose that in the header you provided.
        return 0.0f;
    }

    // Return the active driver pointer (internal or external)
    L298NMotorDriverMega* getDriver() {
        return &_internalDriver;
    }

private:
    // If constructed with pin list, we own this instance.
    L298NMotorDriverMega _internalDriver;

    // motor configuration
    bool _m1reversed = false;
    ControlMode _controlMode = OPEN_LOOP;

    // PID controller for closed-loop control (used for VELOCITY or POSITION modes).
    PIDController _pid;

    // Whether to hold position when stopped (useful for position mode)
    bool _holdPositionWhenStopped = false;

    // runtime targets / state
    double _target = 0.0;
};

#endif // SINGLE_MOTOR_CONTROLLER_HPP
