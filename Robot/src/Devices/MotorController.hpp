// MotorController.hpp
// Unified MotorController supporting both Pololu TB9051FTG carrier (Pololu #2997)
// and a typical L298N H-bridge wiring. Arduino-style API.

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <Arduino.h>
#include "utils/PID.hpp"

/*
  Usage summary:
    - For TB9051 (Pololu) behavior:
        pwm1 = PWM1 pin, pwm2 = PWM2 pin or -1 for single-PWM mode,
        en = EN pin (optional), enb = ENB pin (optional),
        diag = DIAG pin (optional), ocm = OCM analog pin (optional).
    - For L298N typical wiring:
        pwm1 = IN1 (digital), pwm2 = IN2 (digital), en = EN (PWM),
        enb, diag, ocm, occ are optional (OCM typically not present).
    - Choose the driver type in the constructor (DriverType::TB9051 or DriverType::L298N).
*/

class MotorController
{
public:
    enum class DriverType
    {
        TB9051,
        L298N
    };

    // Constructor
    // keep signature similar to prior version; add driverType param (defaults to TB9051).
    MotorController(int pwm1_pin, int pwm2_pin = -1, int en_pin = -1, int enb_pin = -1,
                    int diag_pin = -1, int ocm_pin = -1, int occ_pin = -1, float analog_vref = 5.0f,
                    float kp = 0.0, float ki = 0.0, float kd = 0.0,
                    DriverType driver = DriverType::TB9051)
        : _pwm1_pin(pwm1_pin),
          _pwm2_pin(pwm2_pin),
          _en_pin(en_pin),
          _enb_pin(enb_pin),
          _diag_pin(diag_pin),
          _ocm_pin(ocm_pin),
          _occ_pin(occ_pin),
          _analog_vref(analog_vref),
          _two_pwm_mode(pwm2_pin != -1),
          _pid(kp, ki, kd),
          _driverType(driver)
    {
    }

    // call in setup(); configures pinMode for all provided pins
    void begin()
    {
        // Configure depending on driver type
        if (_driverType == DriverType::TB9051)
        {
            if (_pwm1_pin != -1)
                pinMode(_pwm1_pin, OUTPUT);
            if (_pwm2_pin != -1)
                pinMode(_pwm2_pin, OUTPUT);
            if (_en_pin != -1)
                pinMode(_en_pin, OUTPUT);
            if (_enb_pin != -1)
                pinMode(_enb_pin, OUTPUT);
            if (_diag_pin != -1)
                pinMode(_diag_pin, INPUT_PULLUP); // DIAG active LOW
            if (_ocm_pin != -1)
                pinMode(_ocm_pin, INPUT);
            if (_occ_pin != -1)
                pinMode(_occ_pin, OUTPUT);

            // Default safe states:
            if (_en_pin != -1)
                digitalWrite(_en_pin, LOW); // EN low => high-Z by default
            if (_enb_pin != -1)
                digitalWrite(_enb_pin, HIGH); // ENB high => high-Z by default
            if (_occ_pin != -1)
                digitalWrite(_occ_pin, LOW); // default: remain disabled after over-current
        }
        else // DriverType::L298N
        {
            // Typical L298N wiring: pwm1 -> IN1 (digital), pwm2 -> IN2 (digital), en -> EN (PWM)
            if (_pwm1_pin != -1)
                pinMode(_pwm1_pin, OUTPUT); // IN1
            if (_pwm2_pin != -1)
                pinMode(_pwm2_pin, OUTPUT); // IN2
            if (_en_pin != -1)
                pinMode(_en_pin, OUTPUT); // EN (PWM)
            // L298N modules rarely have DIAG/OCM; if user wired external sensors, keep pins as-is
            if (_diag_pin != -1)
                pinMode(_diag_pin, INPUT);
            if (_ocm_pin != -1)
                pinMode(_ocm_pin, INPUT);
            if (_occ_pin != -1)
                pinMode(_occ_pin, OUTPUT);

            // Default safe states: EN disabled (coast), IN pins low
            if (_en_pin != -1)
                analogWrite(_en_pin, 0);
            if (_pwm1_pin != -1)
                digitalWrite(_pwm1_pin, LOW);
            if (_pwm2_pin != -1)
                digitalWrite(_pwm2_pin, LOW);
            if (_occ_pin != -1)
                digitalWrite(_occ_pin, LOW);
        }
    }

    // setSpeed: accepts -255 .. 255 PWM units
    void setSpeed(int speed)
    {
        // clamp
        if (speed > 255)
            speed = 255;
        if (speed < -255)
            speed = -255;

        if (_driverType == DriverType::TB9051)
        {
            setSpeedTB9051(speed);
        }
        else
        {
            setSpeedL298N(speed);
        }
    }

    // PID-based update (unchanged logic; compatible with both drivers)
    // target: desired speed (units must match 'current' measurement used by PID)
    // current: measured speed
    void update(float target, float current)
    {
        // If there's a DIAG/driver-level fault, do not attempt to drive the motor.
        if (hasFault())
        {
            disable(); // safe state
            return;
        }

        // If target is near zero, stop driving and reset PID to avoid windup.
        if (fabs(target) <= _targetDeadband)
        {
            _pid.reset();
            coast();
            return;
        }

        // Compute PID output (note PID::update(measurement, setpoint))
        double pid_out = _pid.update((double)current, (double)target);
        // Serial.print("PID output: ");
        // Serial.println(pid_out);
        // Convert controller output to PWM units if necessary.
        double pwm_f = pid_out * _controlToPwm;

        // Apply small PWM deadband to avoid chatter
        if (fabs(pwm_f) < _pwmDeadband)
            pwm_f = 0.0;

        // Clip to allowed output range
        if (pwm_f > _outputLimit)
            pwm_f = _outputLimit;
        if (pwm_f < -_outputLimit)
            pwm_f = -_outputLimit;

        int pwm_int = (int)round(pwm_f);
        setSpeed(pwm_int);
    }

    // direct helpers (behavior depends on driver type when appropriate)
    void enable()
    {
        if (_driverType == DriverType::TB9051)
        {
            if (_en_pin != -1)
                digitalWrite(_en_pin, HIGH);
            if (_enb_pin != -1)
                digitalWrite(_enb_pin, LOW);
        }
        else // L298N: enable means PWM EN should be high (we leave PWM duty to setSpeed); just set EN pin high if no PWM needed
        {
            if (_en_pin != -1)
                analogWrite(_en_pin, 255);
        }
    }

    void disable()
    {
        if (_driverType == DriverType::TB9051)
        {
            if (_en_pin != -1)
                digitalWrite(_en_pin, LOW);
            if (_enb_pin != -1)
                digitalWrite(_enb_pin, HIGH);
        }
        else // L298N: coast by leaving EN low (outputs float)
        {
            if (_en_pin != -1)
                analogWrite(_en_pin, 0);
        }
    }

    void brake()
    {
        if (_driverType == DriverType::TB9051)
        {
            // Existing TB9051 brake behaviour: set both PWMs low with EN active
            if (_two_pwm_mode)
            {
                if (_pwm1_pin != -1)
                    analogWrite(_pwm1_pin, 0);
                if (_pwm2_pin != -1)
                    analogWrite(_pwm2_pin, 0);
            }
            else
            {
                if (_pwm1_pin != -1)
                    analogWrite(_pwm1_pin, 0);
                enable();
            }
        }
        else // L298N braking: set IN1 and IN2 equal and EN high to short motor terminals
        {
            if (_pwm1_pin != -1)
                digitalWrite(_pwm1_pin, HIGH);
            if (_pwm2_pin != -1)
                digitalWrite(_pwm2_pin, HIGH);
            if (_en_pin != -1)
                analogWrite(_en_pin, 255); // full enable
        }
    }

    void coast()
    {
        if (_driverType == DriverType::TB9051)
        {
            // TB9051 coast -> disable enables
            if (_en_pin != -1)
                digitalWrite(_en_pin, LOW);
            if (_enb_pin != -1)
                digitalWrite(_enb_pin, HIGH);
            if (_pwm1_pin != -1)
                analogWrite(_pwm1_pin, 0);
            if (_pwm2_pin != -1)
                analogWrite(_pwm2_pin, 0);
        }
        else
        {
            // L298N coast -> disable EN so outputs float
            if (_en_pin != -1)
                analogWrite(_en_pin, 0);
            if (_pwm1_pin != -1)
                digitalWrite(_pwm1_pin, LOW);
            if (_pwm2_pin != -1)
                digitalWrite(_pwm2_pin, LOW);
        }
    }

    // Returns true if DIAG indicates a fault (DIAG is active low on TB9051).
    // For L298N, if no DIAG pin wired, returns false.
    bool hasFault()
    {
        if (_diag_pin == -1)
            return false;

        // For TB9051 DIAG is active-low and used to indicate faults.
        if (_driverType == DriverType::TB9051)
        {
            return (digitalRead(_diag_pin) == LOW);
        }
        else
        {
            // L298N typically has no DIAG. If user provided a diag_pin (e.g., external sensor),
            // interpret LOW as a fault by default.
            return (digitalRead(_diag_pin) == LOW);
        }
    }

    // Attempts to clear latched faults (best-effort). Behavior is TB9051-focused.
    void clearFault()
    {
        if (_driverType == DriverType::TB9051)
        {
            if (_en_pin != -1)
            {
                digitalWrite(_en_pin, LOW);
                delay(5);
                digitalWrite(_en_pin, HIGH);
                delay(5);
            }
            if (_enb_pin != -1)
            {
                digitalWrite(_enb_pin, HIGH);
                delay(5);
                digitalWrite(_enb_pin, LOW);
                delay(5);
            }
        }
        else
        {
            // L298N typically has no latched internal faults - do nothing.
        }
    }

    // Read current in amps. For TB9051 use OCM (if wired). For L298N, return NAN unless an external sensor
    // connected to _ocm_pin provides a voltage to convert.
    float readCurrentAmps()
    {
        if (_ocm_pin == -1)
            return NAN;
        int raw = analogRead(_ocm_pin);
        float volts = _adcToVoltage(raw);

        // TB9051 OCM ~0.5 V/A; many external sensors have different scales.
        if (_driverType == DriverType::TB9051)
        {
            const float volts_per_amp = 0.5f;
            return volts / volts_per_amp;
        }
        else
        {
            // For L298N, treat the analog pin as an external current-sense input that the user must scale.
            // Use _externalSenseVoltsPerAmp (default NAN); if user set to a value, use it, otherwise return NAN.
            if (!isnan(_externalSenseVoltsPerAmp) && _externalSenseVoltsPerAmp > 0.0f)
            {
                return volts / _externalSenseVoltsPerAmp;
            }
            return NAN;
        }
    }

    // Configure whether the two-PWM mode is used (true) or single-PWM + EN mode (false).
    void setTwoPwmMode(bool useTwoPwm) { _two_pwm_mode = useTwoPwm; }

    // Set driver type at runtime
    void setDriverType(DriverType driver) { _driverType = driver; }

    // Setters for tuning
    void setControlToPwm(double scale) { _controlToPwm = scale; }
    void setOutputLimit(double limit) { _outputLimit = fabs(limit); }
    void setPwmDeadband(double db) { _pwmDeadband = fabs(db); }
    void setTargetDeadband(double db) { _targetDeadband = fabs(db); }
    void setExternalSenseVoltsPerAmp(float vpa) { _externalSenseVoltsPerAmp = vpa; } // for L298N external sensors

private:
    // Low-level implementations per driver type
    void setSpeedTB9051(int speed)
    {
        // clamp already done by caller, but be defensive
        if (speed > 255)
            speed = 255;
        if (speed < -255)
            speed = -255;

        if (speed == 0)
        {
            // default: coast
            coast();
            return;
        }

        enable(); // ensure driver enabled for TB9051

        uint8_t duty = (uint8_t)abs(speed);

        if (_two_pwm_mode)
        {
            if (speed > 0)
            {
                if (_pwm1_pin != -1)
                    analogWrite(_pwm1_pin, duty);
                if (_pwm2_pin != -1)
                    analogWrite(_pwm2_pin, 0);
            }
            else
            {
                if (_pwm1_pin != -1)
                    analogWrite(_pwm1_pin, 0);
                if (_pwm2_pin != -1)
                    analogWrite(_pwm2_pin, duty);
            }
        }
        else
        {
            // single PWM + EN mode: apply PWM on pwm1 and use EN for enable
            if (_pwm1_pin != -1)
                analogWrite(_pwm1_pin, duty);
            enable();
        }
    }

    void setSpeedL298N(int speed)
    {
        // L298N standard: IN1/IN2 digital direction (pwm1/pwm2), EN is PWM (en_pin)
        // clamp already done by caller
        if (_pwm1_pin == -1 || _pwm2_pin == -1 || _en_pin == -1)
        {
            // missing mapping for L298N - cannot drive
            return;
        }

        if (speed == 0)
        {
            // coast: disable EN
            analogWrite(_en_pin, 0);
            digitalWrite(_pwm1_pin, LOW);
            digitalWrite(_pwm2_pin, LOW);
            return;
        }

        uint8_t duty = (uint8_t)abs(speed);
        // Serial.print("Duty: ");
        // Serial.println(duty);

        if (speed > 0)
        {
            // Serial.println("Positive Speed");
            digitalWrite(_pwm1_pin, HIGH); // IN1
            digitalWrite(_pwm2_pin, LOW);  // IN2
            analogWrite(_en_pin, duty);    // EN PWM
        }
        else
        {
            digitalWrite(_pwm1_pin, LOW);
            digitalWrite(_pwm2_pin, HIGH);
            analogWrite(_en_pin, duty);
        }
    }

    // Helpers & members
    float _adcToVoltage(int raw)
    {
        return (_analog_vref * raw) / 1023.0f;
    }

    int _pwm1_pin;
    int _pwm2_pin;
    int _en_pin;
    int _enb_pin;
    int _diag_pin;
    int _ocm_pin;
    int _occ_pin;

    PIDController _pid;

    float _analog_vref;
    bool _two_pwm_mode;

    DriverType _driverType;

    // --- PID -> PWM conversion tuning parameters ---
    double _controlToPwm = 1.0;    // multiplier from PID units to PWM units
    double _outputLimit = 255.0;   // clamp magnitude of PWM
    double _pwmDeadband = .05;     // small deadband around zero (PWM units)
    double _targetDeadband = 0.01; // deadband for target considered zero (target units)

    // For external current-sense on L298N or other drivers (volts per amp)
    float _externalSenseVoltsPerAmp = NAN;
};

#endif // MOTOR_CONTROLLER_HPP
