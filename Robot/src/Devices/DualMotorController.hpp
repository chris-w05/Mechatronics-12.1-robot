#ifndef DUAL_MOTOR_CONTROLLER_HPP
#define DUAL_MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "utils/PID.hpp"
#include <DualTB9051FTGMotorShield.h>


//Default pins for the board. When a second board is created, one of these shoudl be created for it
struct TB9051Pins
{
    uint8_t m1en, m1dir, m1pwm, m1diag, m1ocm;
    uint8_t m2en, m2dir, m2pwm, m2diag, m2ocm;
};

class DualMotorController
{
public:
    // Default-pins constructor (shield #1)
    DualMotorController(
        double kp1, double ki1, double kd1, bool m1reversed,
        double kp2, double ki2, double kd2, bool m2reversed,
        bool holdPositionWhenStopped1 = false,
        bool holdPositionWhenStopped2 = false)
        : 
        //dualDriver(), // default pins
          _m1reversed(m1reversed),
          _m2reversed(m2reversed),
          _pid1(kp1, ki1, kd1),
          _pid2(kp2, ki2, kd2),
          _holdPositionWhenStopped1(holdPositionWhenStopped1),
          _holdPositionWhenStopped2(holdPositionWhenStopped2)
    {
    }

    // Remapped-pins constructor (shield #2, #3, ...)
    DualMotorController(
        const TB9051Pins &pins,
        double kp1, double ki1, double kd1, bool m1reversed,
        double kp2, double ki2, double kd2, bool m2reversed,
        bool holdPositionWhenStopped1 = false,
        bool holdPositionWhenStopped2 = false)
        : 
        // dualDriver(
        //       pins.m1en, pins.m1dir, pins.m1pwm, pins.m1diag, pins.m1ocm,
        //       pins.m2en, pins.m2dir, pins.m2pwm, pins.m2diag, pins.m2ocm),
          _m1reversed(m1reversed),
          _m2reversed(m2reversed),
          _pid1(kp1, ki1, kd1),
          _pid2(kp2, ki2, kd2),
          _holdPositionWhenStopped1(holdPositionWhenStopped1),
          _holdPositionWhenStopped2(holdPositionWhenStopped2)
    {
    }

    enum ControlMode{
        POSITION,
        VELOCITY
    };

    void init()
    {
        dualDriver.init();
        dualDriver.enableDrivers();
        Serial.println("Drivetrain Drivers enabled");
        dualDriver.flipM1(_m1reversed);
        dualDriver.flipM2(_m2reversed);
    }

    void setTarget(double targetM1, double targetM2)
    {
        _target1 = targetM1;
        _target2 = targetM2;
    }

    void setPower( int signalM1, int signalM2 ){
        dualDriver.setM1Speed(signalM1);
        dualDriver.setM2Speed(signalM2);
    }

    void update(double current_value1, double current_value2)
    {
        double signal1 = _pid1.update(current_value1, _target1);
        double signal2 = _pid2.update(current_value2, _target2); // <-- FIX

        signal1 = constrain(signal1, -400, 400);
        signal2 = constrain(signal2, -400, 400);


        // Serial.print("Signal L ");
        // Serial.print(signal1);
        // Serial.print("Signal R ");
        // Serial.println(signal2);
        dualDriver.setM1Speed((int)signal1);
        dualDriver.setM2Speed((int)signal2);
    }

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

    float getM1current() { return dualDriver.getM1CurrentMilliamps(); }
    float getM2current() { return dualDriver.getM2CurrentMilliamps(); }

private:
    DualTB9051FTGMotorShield dualDriver;

    bool _m1reversed = false;
    bool _m2reversed = true;
    
    PIDController _pid1;
    PIDController _pid2;

    bool _holdPositionWhenStopped1 = false;
    bool _holdPositionWhenStopped2 = false;

    double _target1 = 0.0;
    double _target2 = 0.0;

    
};

#endif
