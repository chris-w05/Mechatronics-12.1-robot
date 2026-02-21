#pragma once

#include <Arduino.h>
#include "Config.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "Devices/LineSensor.hpp"
#include "Devices/DistanceSensor.hpp"
#include "utils/Odometry.hpp"
#include "Subsystem.h"

class Drive : public Subsystem {

    enum MODE
    {   
        LINEFOLLOWING,
         STRAIGHT,
         ARC,
         STOPPED,
         HARDSET,
         LINEFOLLOWING_HARDSET
    };

    private:
        float _speedL = 0;
        float _speedR = 0;

        EncoderWrapper _leftEncoder;
        EncoderWrapper _rightEncoder;

        LineSensor _lineSensor;

        DualMotorController _motorController;

        SharpGP2Y0A51 distSensor;

        Odometry _odometry;

        MODE mode = HARDSET;


    public:

        /**
         * Drive contructor
         */
        Drive(
            const int left_enc_a,
            const int left_enc_b,
            const int right_enc_a,
            const int right_enc_b,
            const TB9051Pins pins,
            const int distPin,
            const int lineFollowerPin)
            : _leftEncoder(left_enc_a, left_enc_b),
            _rightEncoder(right_enc_a, right_enc_b),
            _lineSensor(lineFollowerPin),
            _motorController(
                pins,
                DRIVE_L_PID, true,
                DRIVE_R_PID, false,
                false, false),
            distSensor( distPin)
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
            _motorController.setPIDFeedForwardFunc(driveFF, driveFF);
            Serial.println("Drivetrain initialized");
        }

        /**
         * Called once per loop of arduino,
         * This is where feedback control is updated
         */
        void update()
        {
            //Update child sensors
            _leftEncoder.update();
            _rightEncoder.update();
            float leftVelocity = _leftEncoder.getVelocity() * DRIVETRAIN_TICKS_TO_IN; //inch/s
            float rightVelocity = _rightEncoder.getVelocity() * DRIVETRAIN_TICKS_TO_IN; //inch/s
            float leftAcceleration = _leftEncoder.getAcceleration() * DRIVETRAIN_TICKS_TO_IN; //in/s^2
            float rightAcceleration = _rightEncoder.getAcceleration() * DRIVETRAIN_TICKS_TO_IN; //in/s^2

            //Apply feedback/ open loop control depending on current mode
            switch( mode){
                case LINEFOLLOWING:{
                    _lineSensor.update();
                    float correction = _lineSensor.readValue();
                    correction *= DRIVE_LINEFOLLOW_VELOCITY_GAIN; // Correction gain - velocity units/number sensors active
                    float newSpeedL = _speedL - correction/LINESENSOR_LR_RATIO;
                    float newSPeedR = _speedR + correction;
                    _motorController.setTarget( newSpeedL, newSPeedR);
                    _motorController.update(leftVelocity, leftAcceleration, rightVelocity, rightAcceleration);
                    break;
                }
                case STRAIGHT:
                case ARC:
                case STOPPED:
                    _motorController.setTarget(_speedL, _speedR);
                    _motorController.update(leftVelocity, leftAcceleration, rightVelocity, rightAcceleration);
                    break;
                case LINEFOLLOWING_HARDSET:{
                    // Serial.print("Commanding speeds Left:");
                    // Serial.print(_speedL);
                    // Serial.print(" Right:");
                    // Serial.println(_speedR);

                    _lineSensor.update();
                    float kp = DRIVE_LINEFOLLOW_GAIN * (_speedL + _speedR)/2;
                    float correction = _lineSensor.readValue();
                    correction *= DRIVE_LINEFOLLOW_GAIN;
                    int leftCmd = _speedL - correction/LINESENSOR_LR_RATIO; //Scale down left side command due to off-center nature of line sensor
                    int rightCmd = _speedR = + correction;
                    _motorController.setPower(leftCmd, rightCmd);
                    break;
                }
                case HARDSET:
                    if (_speedL != 0.0 && _speedR != 0.0){
                        // Serial.print("Velocity L: ");
                        // Serial.print(_leftEncoder.getVelocity());
                        // Serial.print(" targetL: ");
                        // Serial.print(_speedL);
                        Serial.print(" leftVel(inch/s): ");
                        Serial.print(leftVelocity);

                        // Serial.print(" Velocity R: ");
                        // Serial.print(_rightEncoder.getVelocity());
                        // Serial.print(" targetR: ");
                        // Serial.print(_speedR);
                        Serial.print(" rightVel(inch/s): ");
                        Serial.println(rightVelocity);
                    }
                    _motorController.setPower((int)_speedL, (int)_speedR);
                    break;
                default:
                    _motorController.setPower(0, 0);
            }

            _odometry.update(_leftEncoder, _rightEncoder);
        }
        

        /** 
         * Gets the current position and orientation of the robot
         */
        Odometry::Pose2D getPose(){
            return _odometry.getPose();
        }

        float getAccumulatedHeading(){
            return _odometry.getAccumulatedHeading();
        }

        /**
         * Passes total distance travelled from odometry
         */
        float getDistance() { return _odometry.distanceTravelled(); };


        /**
         * Sets the desired speed for driving in a straight line
         * This assumes PID control is being used and accurate
         */
        void setSpeed(float speed){
            mode = STRAIGHT;
            _motorController.resetPID();
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
         * Sets target velocities for feedback control to follow a specified arc
         */
        void followRadiusClockwise(float omega_rad_s, float radius_m)
        {
            mode = MODE::ARC;

            // track half-width
            const float halfL = DRIVETRAIN_WIDTH / 2.0f;

            _speedL = omega_rad_s * (radius_m + halfL);
            _speedR = omega_rad_s * (radius_m - halfL);
        }

        void followRadiusAtVelocity(float velocity, float radius_m)
        {
            mode = MODE::ARC;

            // track half-width
            const float halfL = DRIVETRAIN_WIDTH / 2.0f;
            float omega = (velocity / abs(radius_m));
            bool sign = 2 * (short)(radius_m > 0) - 1;
            _speedL = omega * (radius_m + sign * halfL);
            _speedR = omega * (radius_m - sign * halfL);
        }

        /**
         * Changes mode to LINEFOLLOWING, and gives a target speed for feedback control
         */
        void followLine(float speed)
        {
            mode = MODE::LINEFOLLOWING;
            _speedL = speed;
            _speedR = speed;
        }


        /**
         * Follow a radius in the Counter-clockwise direction
         */
        void followRadiusCCW(float omega_rad_s, float radius_m)
        {
            mode = MODE::ARC;

            // track half-width
            const float halfL = DRIVETRAIN_WIDTH / 2.0f;

            _speedL = omega_rad_s * (radius_m - halfL);
            _speedR = omega_rad_s * (radius_m + halfL);
        }

        /**
         * Closed-loop control for stopping the robot. 
         * Sets the desired velocity to 0, so the robot will actively brake
         * If you want to set power to 0 as a soft-stop, use hardSetSpeed(0);
         */
        void stop()
        {
            mode = MODE::STOPPED;
            setSpeed(0);
        }

};
