#pragma once

#include <Arduino.h>
#include "Config.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "Devices/LineSensor.hpp"
#include "Devices/DistanceSensor.hpp"
#include "utils/Odometry.hpp"
#include "Subsystem.h"
#include "utils/OdometryPlotting.hpp"

static unsigned long lastTelemetryMs = 0;

class Drive : public Subsystem {

    enum MODE
    {   
        LINEFOLLOWING,
         STRAIGHT,
         ARC,
         STOPPED,
         HARDSET,
         LINEFOLLOWING_HARDSET,
         DISTANCE
    };

    private:
        float _speedL = 0;
        float _speedR = 0;
        float targetDistance = 0;

        float leftTargetPosition = 0;
        float rightTargetPosition = 0;
        unsigned long lastTime = 0;

        EncoderWrapper _leftEncoder;
        EncoderWrapper _rightEncoder;

        LineSensor _lineSensor;

        DualMotorController _motorController;

        SharpGP2Y0A51 _distSensor;

        Odometry _odometry;

        MODE mode = HARDSET;

    public:

        /**
         * Drive contructor
         */
        Drive(
            const uint16_t left_enc_a,
            const uint16_t left_enc_b,
            const uint16_t right_enc_a,
            const uint16_t right_enc_b,
            const TB9051Pins pins,
            const uint16_t distPin,
            const uint16_t lineFollowerPins[8])
            : _leftEncoder(left_enc_a, left_enc_b),
              _rightEncoder(right_enc_a, right_enc_b),
              _lineSensor(lineFollowerPins, lineSensorCalMin, lineSensorCalMax),
              _motorController(
                  pins,
                  DRIVE_L_PID, true,
                  DRIVE_R_PID, false,
                  false, false),
              _distSensor(distPin, distanceSensor_VoltageToDistance)
        {
        }


        /**
         * Should be called once on startup
         * Starts odometry, inits sensors, etc. 
         */
        void init()
        {
            _odometry.init({0, 0, 0});
            _leftEncoder.init();
            _rightEncoder.init();
            _leftEncoder.flipDirection();
            _motorController.init();
            _lineSensor.init();
            // _motorController.setPIDDerivativeFeedForwardFunc(drivedFF, drivedFF);

            // _motorController.setPIDFeedForwardFunc(driveFF, driveFF);
            Serial.println("Drivetrain initialized");
            
        }

        /**
         * Called once per loop of arduino,
         * This is where feedback control is updated
         */
        void update()
        {
            //Update child sensors
            unsigned long now = micros();
            float dt = (now -lastTime) * .000001;
            _distSensor.update();
            _leftEncoder.update();
            _rightEncoder.update();
            _lineSensor.update();
            float leftPosition = _leftEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            float rightPosition = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            float leftVelocity = _leftEncoder.getVelocity() * DRIVETRAIN_TICKS_TO_IN; //inch/s
            float rightVelocity = _rightEncoder.getVelocity() * DRIVETRAIN_TICKS_TO_IN; //inch/s
            // float leftAcceleration = _leftEncoder.getAcceleration() * DRIVETRAIN_TICKS_TO_IN; //in/s^2
            // float rightAcceleration = _rightEncoder.getAcceleration() * DRIVETRAIN_TICKS_TO_IN; //in/s^2

            //Apply feedback/ open loop control depending on current mode
            switch( mode){
                case LINEFOLLOWING:{

                    float correction = _lineSensor.getPosition();
                    correction *= DRIVE_LINEFOLLOW_VELOCITY_GAIN * _speedL; // Correction gain - velocity units/number sensors active
                    float newSpeedL = _speedL + correction / LINESENSOR_LR_RATIO;
                    float newSpeedR = _speedR - correction;
                    leftTargetPosition += newSpeedL * dt;
                    rightTargetPosition += newSpeedR * dt;
                    Serial.println(correction);
                    // Serial.print("Left target speed ");
                    // Serial.print(newSpeedL);
                    // Serial.print(" Right target speed");
                    // Serial.println(newSpeedR);
                    _motorController.setTarget(leftTargetPosition, rightTargetPosition);
                    _motorController.update(leftPosition, leftVelocity, rightPosition, rightVelocity, _speedL, _speedR);
                    break;
                }
                case STRAIGHT:
                case ARC:
                case STOPPED:

                    // Limit maximum speeds to be achieveable
                    if (abs(_speedL) > MAXVELOCITY)
                    {
                        float scale = MAXVELOCITY / _speedL;
                        _speedL *= scale;
                        _speedR *= scale;
                    }
                    if (abs(_speedR) > MAXVELOCITY)
                    {
                        float scale = MAXVELOCITY / _speedR;
                        _speedL *= scale;
                        _speedR *= scale;
                    }

                    leftTargetPosition += _speedL * dt;
                    rightTargetPosition += _speedR * dt;
                    // Position based control
                    _motorController.setTarget(leftTargetPosition, rightTargetPosition);
                    _motorController.update(leftPosition, leftVelocity, rightPosition, rightVelocity, _speedL, _speedR);

                    
                    if (millis() - lastTelemetryMs >= 50) // 20 Hz
                    {
                        lastTelemetryMs = millis();
                        //Log the odometry
                        emitTelemetryJSON(
                            Serial,
                            _odometry,
                            leftVelocity,
                            rightVelocity,
                            leftPosition,
                            rightPosition,
                            leftTargetPosition,
                            rightTargetPosition
                        );
                    }

                    // // Velocity based control:
                    //  _motorController.setTarget(_speedL, _speedR);
                    //  _motorController.update(leftVelocity, leftAcceleration, rightVelocity, rightAcceleration);
                    break;
                case LINEFOLLOWING_HARDSET:{
                    

                    _lineSensor.update();
                    float correction = _lineSensor.getPosition();
                    Serial.println(correction);
                    correction *= DRIVE_LINEFOLLOW_GAIN * (_speedL +_speedR)/2;
                    int leftCmd = _speedL + correction / LINESENSOR_LR_RATIO; // Scale down left side command due to off-center nature of line sensor
                    int rightCmd = _speedR  - correction;
                    _motorController.setPower(leftCmd, rightCmd);
                    break;
                }
                case HARDSET:
                    if (_speedL != 0.0 && _speedR != 0.0){

                        // Serial.print(">SpeedL:");
                        // Serial.print(_speedL);
                        // Serial.print(",VelocityL:");
                        // Serial.print(leftVelocity);
                        // Serial.print(",SpeedR:");
                        // Serial.print(_speedR);
                        // Serial.print(",VelocityR:");
                        // Serial.print(rightVelocity);
                        // Serial.println("\r");
                    }
                    _motorController.setPower((int)_speedL, (int)_speedR);
                    break;

                case DISTANCE:
                {
                    float distance = _distSensor.getDistanceCm();;
                    // Serial.print("Error");
                    // Serial.print(error);
                    // Serial.print(" ");
                    // Serial.print(" Left:");
                    // Serial.print(_speedL);
                    // Serial.print(" Right:");
                    // Serial.print(_speedR);
                    // Serial.print(" leftVel(inch/s): ");
                    // Serial.print(leftVelocity);
                    // Serial.print(" rightVel(inch/s): ");
                    // Serial.println(rightVelocity);

                    _motorController.setTarget(targetDistance, targetDistance);
                    _motorController.updateWithoutDerivatice(distance, distance);
                    break;
                }
                default:
                    _motorController.setPower(0, 0);
            }

            lastTime = now;
            _odometry.update(_leftEncoder, _rightEncoder);
        }
        

        /** 
         * Gets the current position and orientation of the robot
         */
        Odometry::Pose2D getPose(){
            return _odometry.getPose();
        }

        /**
         * Passes the current distance reading from the distance sensor
         */
        float getDistanceSensorReading(){
            return _distSensor.getDistanceCm();
        }

        /**
         * Passes the current heading, not modded by PI (range is -inf -> inf)
         */
        float getAccumulatedHeading(){
            return _odometry.getAccumulatedHeading();
        }

        /**
         * Passes total distance travelled from odometry
         */
        float getDistance() { return _odometry.distanceTravelled(); };

        /**
         * returns the average of wheel velocities
         */
        float getAvgVelocity(){
            return (_leftEncoder.getVelocity() + _rightEncoder.getVelocity()) * DRIVETRAIN_TICKS_TO_IN / 2;
        }


        /**
         * Sets the desired speed for driving in a straight line
         * This assumes PID control is being used and accurate
         * @param speed Target speed for the drivetrain to drive in a straight line
         * @param resetPID If this is set to true, the PID controller will be reset. i.e. the integral value will be set to 0, but all gains will be retained
         */
        void setSpeed(float speed, bool resetPID = true){
            mode = STRAIGHT;
            // _motorController.setPID(DRIVE_L_PID, DRIVE_R_PID);
            if( resetPID ){
                _motorController.resetPID();
            }
            
            leftTargetPosition = _leftEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            rightTargetPosition = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            _speedR = speed;
            _speedL = speed;
        }


        /**
         * Sends a raw speed command (no feedback control) to both motors
         * This will make the robot drive in a straight-ish line
         */
        void hardSetSpeed(int16_t speed){
            mode = HARDSET;
            _speedR = speed;
            _speedL = speed;
            _motorController.setPower(speed, speed);
        }


        /**
         * Individually set the speeds of both drive motors
         * This is done outside of feedback control
         */
        void hardSetSpeed(int16_t speed1, int16_t speed2)
        {
            mode = HARDSET;
            _speedR = speed1;
            _speedL = speed2;
            _motorController.setPower(speed1, speed2);
        }


        /**
         * Sets target velocities for feedback control to follow a specified arc at a specified angular velocity
         * 
         * @param omega_rad_s The angular velocity of the robot following the curve, in radians/s
         * @param radius The radius of the curve to follow, in inches
         */
        void followRadiusClockwise(float omega_rad_s, float radius)
        {
            mode = MODE::ARC;
            // _motorController.setPID(DRIVE_L_PID, DRIVE_R_PID);
            _motorController.makeLinear();
            // track half-width
            const float halfL = DRIVETRAIN_WIDTH / 2.0f;

            leftTargetPosition = _leftEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            rightTargetPosition = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            _speedL = omega_rad_s * (radius + halfL);
            _speedR = omega_rad_s * (radius - halfL);
        }

        /**
         * Sets target velocities for feedback control to follow a specified arc at a specific tangential velocity
         * 
         * @param velocity The tangential velocity to follow the arc at
         * @param radius The radius of the arc for the robot to follow
         */
        void followRadiusAtVelocity(float velocity, float radius, bool resetPID = true)
        {
            mode = MODE::ARC;
            // _motorController.setPID(DRIVE_L_PID, DRIVE_R_PID);
            if (resetPID)
            {
                _motorController.resetPID();
            }
            

            // track half-width
            const float halfL = DRIVETRAIN_WIDTH / 2.0f;

            // initialize target positions at current encoder readings (unchanged)
            leftTargetPosition = _leftEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            rightTargetPosition = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;

            // Handle special case: radius == 0 (spin in place).
            // Reasonable convention chosen: for radius == 0 we perform an in-place spin
            // where positive velocity results in rightward (clockwise) spin on the robot.
            // (If you prefer the opposite, flip the signs below.)
            if (abs(radius) < 1e-6f)
            {
                // Make left and right opposite to spin in place.
                // Here: left backward, right forward for a clockwise spin
                _speedL = -velocity;
                _speedR = velocity;
                return;
            }

            // Compute angular velocity. Using the sign of radius:
            // - positive radius => turn right (clockwise) => negative ω (we apply a - sign).
            // - negative radius => turn left (counter-clockwise) => positive ω.
            float omega = -velocity / radius; // no abs() so sign of radius matters

            // Convert centre tangential velocity + angular velocity to wheel linear speeds:
            _speedL = velocity - halfL * omega;
            _speedR = velocity + halfL * omega;
        }

        /**
         * Changes mode to LINEFOLLOWING, and gives a target speed for feedback control
         * 
         * @param speed The speed in in/s for the robot to drive at
         */
        void followLine(float speed)
        {
            mode = MODE::LINEFOLLOWING;
            // _motorController.setPID(DRIVE_L_PID, DRIVE_R_PID);
            _motorController.makeLinear();

            leftTargetPosition = _leftEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            rightTargetPosition = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            _speedL = speed;
            _speedR = speed;
        }

        /**
         * Changes mode to LINEFOLLOWING_HARDSET, and gives a target speed for feedback control
         * 
         * @param speed The signal to send the motor from (-400 to 400). Values outside this range will be constrained by the motor controller
         */
        void followLineHardset(int speed)
        {
            mode = MODE::LINEFOLLOWING_HARDSET;
            _speedL = speed;
            _speedR = speed;
        }

        /**
         * Follow a radius in the Counter-clockwise direction
         */
        void followRadiusCCW(float omega_rad_s, float radius)
        {
            mode = MODE::ARC;
            // _motorController.setPID(DRIVE_L_PID, DRIVE_R_PID);
            _motorController.makeLinear();

            // track half-width
            const float halfL = DRIVETRAIN_WIDTH / 2.0f;

            leftTargetPosition = _leftEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            rightTargetPosition = _rightEncoder.getCount() * DRIVETRAIN_TICKS_TO_IN;
            _speedL = omega_rad_s * (radius - halfL);
            _speedR = omega_rad_s * (radius + halfL);
        }

        /**
         * Have the robot hold a specific distance from the wall
         * 
         * @param distance The distance for the robot to hold from the wall
         */
        void apporachDistance(float distance){
            mode = MODE::DISTANCE;
            // _motorController.setPID(DRIVE_DISTANCE_PID, DRIVE_DISTANCE_PID);
            // _motorController.setPIDKpFunction(driveNonlinearP, driveNonlinearP);
            targetDistance = distance;
            
            _speedL = 0;
            _speedR = 0;
        }


        /**
         * Set the value for a PID constant 
         */
        void setDriveMotorPIDConstant(float value, float index){
            _motorController.setPIDConstant(value, index);
        }

        /**
         * Closed-loop control for stopping the robot. 
         * Sets the desired velocity to 0, so the robot will actively brake
         * If you want to set power to 0 as a soft-stop, use hardSetSpeed(0);
         */
        void stop()
        {
            mode = MODE::STOPPED;
            // _motorController.setPID(DRIVE_L_PID, DRIVE_R_PID);
            hardSetSpeed(0);
        }

};
