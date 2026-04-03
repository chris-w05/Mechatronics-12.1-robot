/**
 * @file Drive.hpp
 * @brief Differential-drive subsystem: closed-loop wheel control, line following,
 *        wall-distance holding, and odometry integration.
 *
 * The Drive subsystem manages all motion of the robot as a differential drive.
 * It supports several operating modes (see Drive::MODE) selected by calling a
 * set-point method, then calling `update()` every loop iteration to execute
 * closed-loop control and advance the odometry estimate.
 *
 * Both an encoder-derived (true) pose and a feedforward-derived (desired) pose
 * are maintained so that trajectory-tracking error can be computed.  The
 * RamseteController optionally corrects heading drift in STRAIGHT and ARC modes.
 */
#pragma once

#include <Arduino.h>
#include "Config.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "Devices/LineSensor.hpp"
#include "Devices/DistanceSensor.hpp"
#include "utils/Odometry.hpp"
#include "utils/RamseteController.hpp"
#include "utils/OdometryPlotting.hpp"
#include "Subsystem.h"

/**
 * Differential-drive subsystem.
 *
 * Supports several drive modes (see MODE enum). Call one of the
 * set-point methods (setSpeed, followRadiusAtVelocity, etc.) to
 * change mode, then call update() every loop iteration.
 */
class Drive : public Subsystem {


    // =========================================================================
    // Drive mode
    // =========================================================================
    enum MODE {
        STRAIGHT,              ///< Closed-loop straight-line drive
        ARC,                   ///< Closed-loop arc/curve drive
        STOPPED,               ///< Actively braking (target velocity = 0)
        LINEFOLLOWING,         ///< Line following by commanding velocities
        LINEFOLLOWING_HARDSET, ///< Line following using motor signals
        LINEFOLLOWING_DISTANCE,///< Simulataneous line following and distance follwing using motor signals
        HARDSET,               ///< Open-loop: raw left/right motor signals
        DISTANCE               ///< Holds a fixed distance from a wall

    };

private:
    // =========================================================================
    // Velocity state (all speeds in in/s, signals in raw motor units)
    // =========================================================================
    float    _targetSpeedL = 0;  ///< Desired closed-loop speed, left wheel
    float    _targetSpeedR = 0;  ///< Desired closed-loop speed, right wheel
    float    _speedL       = 0;  ///< Ramped (actual) setpoint, left wheel
    float    _speedR       = 0;  ///< Ramped (actual) setpoint, right wheel
    int16_t  _signalL      = 0;  ///< Raw motor signal for HARDSET modes
    int16_t  _signalR      = 0;

    float _targetDistance  = 0;  ///< Distance setpoint for DISTANCE mode (cm)
    float _leftTargetPos   = 0;  ///< Integrated position target, left wheel (in)
    float _rightTargetPos  = 0;  ///< Integrated position target, right wheel (in)

    unsigned long _lastTime = 0;

    // =========================================================================
    // Devices
    // =========================================================================
    EncoderWrapper      _leftEncoder;
    EncoderWrapper      _rightEncoder;
    LineSensor          _lineSensor;
    DualMotorController _motorController;
    SharpGP2Y0A51       _distSensor;

    // =========================================================================
    // Odometry
    // =========================================================================
    Odometry _odometry;        ///< True pose, integrated from encoder ticks
    Odometry _desiredOdometry; ///< Feedforward pose, integrated from target speeds

    // =========================================================================
    // Control
    // =========================================================================
    MODE              _mode            = HARDSET;
    RamseteController _ramsete;               ///< Default tuning: b = 0.006, zeta = 0.9
    PIDController     _distanceSensorPID;     ///< PID controller for driving using distance sensor - This is at the drive level so its signal can be added to _lineSensorPID's signal
    PIDController     _lineSensorPID;         ///< PID controller for line following
    unsigned long     _lastTelemetryMs = 0;   ///< Throttle telemetry output to 20 Hz
    bool              _resetTargetPosBetweenSegments    = false;    ///< Changes whether syncTargetPositions() is called between trajectories

    // =========================================================================
    // Internal helpers
    // =========================================================================

    /** Sync position targets to current encoder positions to avoid jumps on mode switch. */
    void syncTargetPositions()
    {
        _leftTargetPos  = _leftEncoder.getCount()  * DRIVETRAIN_TICKS_TO_IN;
        _rightTargetPos = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
    }

public:
    // =========================================================================
    // Construction / lifecycle
    // =========================================================================

    Drive(const uint16_t left_enc_a,
          const uint16_t left_enc_b,
          const uint16_t right_enc_a,
          const uint16_t right_enc_b,
          const TB9051Pins pins,
          const uint16_t distPin,
          const uint16_t lineFollowerPins[8])
        : _leftEncoder(left_enc_a, left_enc_b),
          _rightEncoder(right_enc_a, right_enc_b),
          _lineSensor(lineFollowerPins, LINESENSORCALMIN, LINESENSORCALMAX),
          _motorController(pins,
                           DRIVE_L_PID, true,
                           DRIVE_R_PID, false,
                           false, false),
          _distSensor(distPin, distanceSensor_VoltageToDistance)
    {}

    /** Called once on startup. Initializes sensors and resets odometry. */
    void init() override
    {
        //Robot starts with odometry point 2 in from wall, in the middle of the start corral
        _odometry.init({2, 6, 0});
        _desiredOdometry.init({2, 6, 0});
        _leftEncoder.init();
        _rightEncoder.init();
        _leftEncoder.flipDirection();
        _motorController.init();
        _lineSensor.init();
        _lastTime = micros();
        _distanceSensorPID.set(DRIVE_DISTANCE_PID);
        _distanceSensorPID.setFilterStrength(1.0);
        _lineSensorPID.set(DRIVE_LINEFOLLOW_GAINS);
        Serial.println("Drivetrain initialized");
        
        // _ramsete.disable();
    }

    /** Called every loop iteration. Updates sensors and applies control. */
    void update() override
    {
        unsigned long now = micros();
        float dt = (now - _lastTime) * 1e-6f;
        _lastTime = now;

        // --- Update sensors ---
        // Encoders are always needed for odometry and PID.
        _leftEncoder.update();
        _rightEncoder.update();
        // Gate expensive peripheral reads to the modes that actually use them.
        // The QTR RC line sensor blocks for up to its timeout (default 1000 µs) per read;
        // the distance sensor adds ~100 µs for analogRead().
        if (_mode == LINEFOLLOWING || _mode == LINEFOLLOWING_HARDSET || _mode == LINEFOLLOWING_DISTANCE)
            _lineSensor.update();
        if (_mode == DISTANCE || _mode == LINEFOLLOWING_DISTANCE)
            _distSensor.update();

        float leftPos  = _leftEncoder.getCount()    * DRIVETRAIN_TICKS_TO_IN;
        float rightPos = _rightEncoder.getCount()   * DRIVETRAIN_TICKS_TO_IN;
        float leftVel  = _leftEncoder.getVelocity() * DRIVETRAIN_TICKS_TO_IN; // in/s
        float rightVel = _rightEncoder.getVelocity()* DRIVETRAIN_TICKS_TO_IN;

        // --- Mode-specific control ---
        switch (_mode) {

            case LINEFOLLOWING: {
                float correction = _lineSensor.getPosition()
                                   * DRIVE_LINEFOLLOW_VELOCITY_GAIN * _speedL;
                float newSpeedL = _speedL + correction / LINESENSOR_LR_RATIO;
                float newSpeedR = _speedR - correction;

                _leftTargetPos  += newSpeedL * dt;
                _rightTargetPos += newSpeedR * dt;

                _motorController.setTarget(_leftTargetPos, _rightTargetPos);
                _motorController.update(leftPos, leftVel, rightPos, rightVel,
                                        _speedL, _speedR);
                break;
            }

            case STRAIGHT:
            case ARC:
            case STOPPED: {
                // Ramp the actual setpoint toward the target to limit acceleration
                const float maxDelta = 3000.0f * dt;
                float deltaL = _targetSpeedL - _speedL;
                float deltaR = _targetSpeedR - _speedR;
                float maxChange = max(fabsf(deltaL), fabsf(deltaR));

                if (maxChange > maxDelta) {
                    float scale = maxDelta / maxChange;
                    _speedL += deltaL * scale;
                    _speedR += deltaR * scale;
                } else {
                    _speedL = _targetSpeedL;
                    _speedR = _targetSpeedR;
                }

                // Clamp feedforward wheel speeds
                float vL_ref = constrain(_speedL, -MAXVELOCITY, MAXVELOCITY);
                float vR_ref = constrain(_speedR, -MAXVELOCITY, MAXVELOCITY);

                // Feedforward chassis velocities
                float vRef = 0.5f * (vL_ref + vR_ref);
                float wRef = (vR_ref - vL_ref) / DRIVETRAIN_WIDTH;

                // Ramsete correction toward the feedforward pose
                float vCmd = vRef, wCmd = wRef;
                if (_ramsete.isEnabled()) {
                    RamseteController::Cmd cmd = _ramsete.step(
                        _desiredOdometry.getPose(), vRef, wRef,
                        _odometry.getPose());
                    vCmd = cmd.v;
                    wCmd = cmd.w;
                }

                // Convert chassis → individual wheel speeds
                float halfW  = DRIVETRAIN_WIDTH * 0.5f;
                float vL_cmd = constrain(vCmd - halfW * wCmd, -MAXVELOCITY, MAXVELOCITY);
                float vR_cmd = constrain(vCmd + halfW * wCmd, -MAXVELOCITY, MAXVELOCITY);

                _leftTargetPos  += vL_cmd * dt;
                _rightTargetPos += vR_cmd * dt;

                _motorController.setTarget(_leftTargetPos, _rightTargetPos);
                _motorController.update(leftPos, leftVel, rightPos, rightVel,
                                        vL_cmd, vR_cmd);

                // Emit telemetry at 20 Hz for real-time plotting / debugging
                // if (millis() - _lastTelemetryMs >= 50) {
                //     _lastTelemetryMs = millis();
                //     emitTelemetryJSON(
                //         Serial,
                //         _odometry, _desiredOdometry,
                //         leftVel,  rightVel,
                //         vL_cmd,   vR_cmd,
                //         leftPos,  rightPos,
                //         _leftTargetPos, _rightTargetPos);
                // }

                // Advance the feedforward (ideal) trajectory
                _desiredOdometry.update(_targetSpeedL * dt, _targetSpeedR * dt);
                break;
            }

            case LINEFOLLOWING_HARDSET: {
                float correction = _lineSensor.getPosition();
                float correction_signal = _lineSensorPID.update(correction, 0); // Scale signal by how fast the robot is moving
                _motorController.setPower(
                    _signalL + (int)(correction_signal / LINESENSOR_LR_RATIO), // Accounts for offcenter position of line sensor
                    _signalR - (int)correction_signal);
                break;
            }

            case LINEFOLLOWING_DISTANCE:{
                //Distance sensor sets the base signal for control:
                float dist = _distSensor.getDistanceIn();
                float signalBase = _distanceSensorPID.update(dist, -(leftVel + rightVel)/2, _targetDistance);
                _signalL = signalBase;
                _signalR = signalBase;
                //Line sensor applies steering correction to distance sensor commands
                float correction = _lineSensor.getPosition();
                float correction_signal = _lineSensorPID.update(correction, 0);     //Scale signal by how fast the robot is moving
                _motorController.setPower(
                    _signalL + (int)(correction_signal / LINESENSOR_LR_RATIO),      //Accounts for offcenter position of line sensor
                    _signalR - (int)correction_signal);
                break;
            }

            case HARDSET:
                _motorController.setPower(_signalL, _signalR);
                break;

            case DISTANCE: {
                // Distance sensor sets the base signal for control:
                float dist = _distSensor.getDistanceIn();
                float signalBase = _distanceSensorPID.update(dist, _targetDistance);
                _signalL = signalBase;
                _signalR = signalBase;
                _motorController.setPower(
                    _signalL, // Accounts for offcenter position of line sensor
                    _signalR);
                break;
            }

            default:
                _motorController.setPower(0, 0);
        }

        // Update true-pose odometry from encoder ticks
        _odometry.update(_leftEncoder, _rightEncoder);
    }

    // =========================================================================
    // Public API — setpoints
    // =========================================================================

    /**
     * Drive straight at the given speed using closed-loop (PID) control.
     * @param speed    Target speed (in/s)
     * @param resetPID If true, resets PID integrators before starting
     */
    void setSpeed(float speed, bool resetPID = true)
    {
        _mode = STRAIGHT;
        if (resetPID) _motorController.resetPID();
        if (_resetTargetPosBetweenSegments)
        {
            syncTargetPositions();
        }
        _targetSpeedL = speed;
        _targetSpeedR = speed;
    }



    /**
     * Drive straight using raw (open-loop) motor signals. No encoder feedback.
     * @param speed Raw signal for both motors
     */
    void hardSetSpeed(int16_t speed)
    {
        _mode    = HARDSET;
        _signalL = speed;
        _signalR = speed;
        _motorController.setPower(speed, speed);
    }

    /**
     * Set independent raw motor signals. No encoder feedback.
     */
    void hardSetSpeed(int16_t left, int16_t right)
    {
        _mode    = HARDSET;
        _signalL = left;
        _signalR = right;
        _motorController.setPower(left, right);
    }

    /**
     * Follow a clockwise arc at a given angular velocity.
     * @param omega_rad_s Angular velocity of the robot (rad/s)
     * @param radius      Arc radius (inches)
     */
    void followRadiusClockwise(float omega_rad_s, float radius)
    {
        _mode = MODE::ARC;
        _motorController.makeLinear();
        if (_resetTargetPosBetweenSegments)
        {
            syncTargetPositions();
        }
        const float halfL = DRIVETRAIN_WIDTH / 2.0f;
        _targetSpeedL = omega_rad_s * (radius + halfL);
        _targetSpeedR = omega_rad_s * (radius - halfL);
    }

    /**
     * Follow an arc at a given tangential (center) velocity.
     * Positive radius turns right (clockwise); negative turns left; zero spins in place.
     * @param velocity  Tangential velocity at the robot center (in/s)
     * @param radius    Arc radius (inches, signed)
     * @param resetPID  If true, resets PID integrators before starting
     */
    void followRadiusAtVelocity(float velocity, float radius, bool resetPID = true)
    {
        _mode = MODE::ARC;
        if (resetPID) _motorController.resetPID();
        if (_resetTargetPosBetweenSegments){            
            syncTargetPositions();
        }

        const float halfL = DRIVETRAIN_WIDTH / 2.0f;

        if (fabsf(radius) < 1e-6f) {
            // Spin in place: positive velocity → clockwise
            _targetSpeedL = -velocity;
            _targetSpeedR =  velocity;
            return;
        }

        float omega   = -velocity / radius; // sign of radius determines turn direction
        _targetSpeedL = velocity - halfL * omega;
        _targetSpeedR = velocity + halfL * omega;
    }

    /**
     * Follow a counter-clockwise arc at a given angular velocity.
     * @param omega_rad_s Angular velocity of the robot (rad/s)
     * @param radius      Arc radius (inches)
     */
    void followRadiusCCW(float omega_rad_s, float radius)
    {
        _mode = MODE::ARC;
        _motorController.makeLinear();
        if (_resetTargetPosBetweenSegments)
        {
            syncTargetPositions();
        }
        const float halfL = DRIVETRAIN_WIDTH / 2.0f;
        _targetSpeedL = omega_rad_s * (radius - halfL);
        _targetSpeedR = omega_rad_s * (radius + halfL);
    }

    /**
     * Follow the line sensor at the given speed using closed-loop control.
     * @param speed Target drive speed (in/s)
     */
    void followLine(float speed)
    {
        _mode = MODE::LINEFOLLOWING;
        _lineSensorPID.reset();
        _motorController.makeLinear();
        if (_resetTargetPosBetweenSegments)
        {
            syncTargetPositions();
        }
        _speedL = speed;
        _speedR = speed;
    }

    /**
     * Follow the line sensor using raw open-loop motor signals (no PID).
     * @param speed Raw motor signal (-400 to 400)
     */
    void followLineHardset(int speed)
    {
        _mode    = MODE::LINEFOLLOWING_HARDSET;
        _lineSensorPID.reset();
        _signalL = speed;
        _signalR = speed;
    }

    /**
     * Hold the robot at a fixed distance from a wall using the distance sensor.
     * @param distance Target distance (cm)
     */
    void approachDistance(float distance)
    {
        _mode           = MODE::DISTANCE;
        _distanceSensorPID.reset();
        _targetDistance = distance;
    }

    /**
     * @brief Follow the line sensor while simultaneously holding a fixed wall distance.
     *
     * The IR distance sensor provides a base forward speed; the line sensor
     * adds a steering correction on top of that signal.
     * @param distance  Target wall-to-robot-edge distance (cm).
     */
    void approachAlongLine(float distance){
        _mode           = MODE::LINEFOLLOWING_DISTANCE;
        _lineSensorPID.reset();
        _distanceSensorPID.reset();
        _targetDistance = distance;
    }

    /**
     * Actively stop the robot by driving target velocity to zero.
     * For an immediate power-off stop, use hardSetSpeed(0) instead.
     */
    void stop() override
    {
        _mode = MODE::STOPPED;
        hardSetSpeed(0);
    }

    /**
     * Set a single PID constant on the motor controller.
     * @param value New constant value
     * @param index Constant index (see DualMotorController)
     */
    void setDriveMotorPIDConstant(float value, int index)
    {
        _motorController.setPIDConstant(value, index);
    }

    /**
     * Toggle whether the ramsete controller is enabled/disabled
     */
    void toggleRamseteCorrection(){
        if(_ramsete.isEnabled()){
            _ramsete.disable();
        }
        else{
            _ramsete.enable();
        }
    }

    // =========================================================================
    // Public API — state queries
    // =========================================================================

    /** Ideal pose derived from the feedforward (target) trajectory. */
    Odometry::Pose2D getPose()             { return _desiredOdometry.getPose(); }

    /** Best-estimate robot pose integrated from encoder ticks. */
    Odometry::Pose2D getTruePose()         { return _odometry.getPose(); }

    /** Ramped (actual) left-wheel velocity setpoint (in/s). */
    float getRampedSpeedL()                { return _speedL; }

    /** Ramped (actual) right-wheel velocity setpoint (in/s). */
    float getRampedSpeedR()                { return _speedR; }

    /** Total path distance from the feedforward trajectory (inches). */
    float getDistance()                    { return _desiredOdometry.distanceTravelled(); }

    /** Total path distance from the encoder-integrated odometry (inches). */
    float getTrueDistance()                { return _odometry.distanceTravelled(); }

    /** Net accumulated heading from the feedforward trajectory (radians). */
    float getAccumulatedHeading()          { return _desiredOdometry.getAccumulatedHeading(); }

    /** Net accumulated heading from the encoder-integrated odometry (radians). */
    float getTrueAccumulatedHeading()      { return _odometry.getAccumulatedHeading(); }

    /** Average of both wheel velocities (in/s). */
    float getAvgVelocity()
    {
        return (_leftEncoder.getVelocity() + _rightEncoder.getVelocity())
               * DRIVETRAIN_TICKS_TO_IN / 2.0f;
    }

    /** Most recent distance-sensor reading (In). */
    float getDistanceSensorReading()       { return _distSensor.getDistanceIn(); }
};
