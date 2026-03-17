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
        Odometry _desiredOdometry;

        MODE mode = HARDSET;

        // Ramsete parameters (good defaults)
        float _ramseteB = 2.0f;
        float _ramseteZeta = 0.7f;
        bool _useRamsete = true; // enable/disable easily

        static float wrapPi(float a)
        {
            while (a > M_PI)
                a -= 2.0f * M_PI;
            while (a < -M_PI)
                a += 2.0f * M_PI;
            return a;
        }

        static float sinc(float x)
        {
            if (fabs(x) < 1e-4f)
                return 1.0f - x * x / 6.0f; // small-angle approx
            return sinf(x) / x;
        }

        struct ChassisCmd
        {
            float v; // in/s
            float w; // rad/s
        };

        /**
         * Increment the ramsete trajectory. 
         * The purpose of this is to get the drrivetrain to correct itself to achieve a desired pose.
         * @param ref pointer to desired pose
         * @param vRef the current feedforward drivetrain velocity ( the "blind" trajectory velocity)
         * @param wRef the current angular velocity of the chassis ("blind rotational velocity")
         * @param cur the current true (best estimate from encoders assuming 0 slip condition) pose of the robot
         */
        ChassisCmd ramseteStep(const Odometry::Pose2D &ref,
                               float vRef, float wRef,
                               const Odometry::Pose2D &cur)
        {
            // error in world
            float dx = ref.x - cur.x;
            float dy = ref.y - cur.y;

            // rotate into robot frame (cur frame)
            float c = cosf(cur.heading);
            float s = sinf(cur.heading);
            float ex = c * dx + s * dy;
            float ey = -s * dx + c * dy;
            float etheta = wrapPi(ref.heading - cur.heading);

            float k = 2.0f * _ramseteZeta * sqrtf(wRef * wRef + _ramseteB * vRef * vRef);

            ChassisCmd out;
            out.v = vRef * cosf(etheta) + k * ex;
            out.w = wRef + k * etheta + _ramseteB * vRef * sinc(etheta) * ey;
            return out;
        }


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
            const int lineFollowerPins[8])
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
            _desiredOdometry.init({0,0,0});
            _leftEncoder.init();
            _rightEncoder.init();
            _leftEncoder.flipDirection();
            _motorController.init();
            _lineSensor.init();
            // _motorController.setPIDDerivativeFeedForwardFunc(drivedFF, drivedFF);
            lastTime = micros();
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
            lastTime = now;
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

            float deltaLeftTargetPosition = 0;
            float deltaRightTargetPosiiton = 0;

            //Apply feedback/ open loop control depending on current mode
            switch( mode){
                case LINEFOLLOWING:{

                    float correction = _lineSensor.getPosition();
                    correction *= DRIVE_LINEFOLLOW_VELOCITY_GAIN * _speedL; // Correction gain - velocity units/number sensors active
                    float newSpeedL = _speedL + correction / LINESENSOR_LR_RATIO;
                    float newSpeedR = _speedR - correction;

                    deltaLeftTargetPosition = newSpeedL *dt;
                    deltaRightTargetPosiiton = newSpeedR *dt;

                    leftTargetPosition += deltaLeftTargetPosition;
                    rightTargetPosition += deltaRightTargetPosiiton;

                    // Serial.println(correction);
                    // Serial.print("Left target speed ");
                    // Serial.print(newSpeedL);
                    // Serial.print(" Right target speed");
                    // Serial.println(newSpeedR);
                    _motorController.setTarget(leftTargetPosition, rightTargetPosition);
                    _motorController.update(leftPosition, leftVelocity, rightPosition, rightVelocity);
                    break;
                }
                case STRAIGHT:
                case ARC:
                case STOPPED:{

                    // RAMSETE TRAJECTORY FOLLOWING ----------------------------------------------------------------------------------------------
                    // Reference wheel speeds from your profile (feedforward)
                    float vL_ref = _speedL;
                    float vR_ref = _speedR;

                    // Clamp refs to achievable wheel speed
                    vL_ref = constrain(vL_ref, -MAXVELOCITY, MAXVELOCITY);
                    vR_ref = constrain(vR_ref, -MAXVELOCITY, MAXVELOCITY);

                    // Reference chassis speeds
                    float vRef = 0.5f * (vL_ref + vR_ref);
                    float wRef = (vR_ref - vL_ref) / DRIVETRAIN_WIDTH;

                    // Get poses
                    auto curPose = _odometry.getPose();
                    auto refPose = _desiredOdometry.getPose();

                    // Ramsete gives corrected chassis commands
                    float vCmd = vRef;
                    float wCmd = wRef;

                    if (_useRamsete)
                    {
                        ChassisCmd cmd = ramseteStep(refPose, vRef, wRef, curPose);
                        vCmd = cmd.v;
                        wCmd = cmd.w;
                    }

                    // Convert commanded chassis speeds to commanded wheel speeds
                    float halfW = DRIVETRAIN_WIDTH * 0.5f;
                    float vL_cmd = vCmd - halfW * wCmd;
                    float vR_cmd = vCmd + halfW * wCmd;

                    // Clamp commanded wheel speeds
                    vL_cmd = constrain(vL_cmd, -MAXVELOCITY, MAXVELOCITY);
                    vR_cmd = constrain(vR_cmd, -MAXVELOCITY, MAXVELOCITY);

                    // Integrate commanded speeds into position targets 
                    deltaLeftTargetPosition = vL_cmd * dt;
                    deltaRightTargetPosiiton = vR_cmd * dt;

                    leftTargetPosition += deltaLeftTargetPosition;
                    rightTargetPosition += deltaRightTargetPosiiton;

                    // Wheel position controller, with dSetpoint = wheel velocity feedforward
                    _motorController.setTarget(leftTargetPosition, rightTargetPosition);


                    
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
                    _motorController.update(leftPosition, leftVelocity, rightPosition, rightVelocity,
                                            vL_cmd, vR_cmd);
                    //Update the ramsete target odometry
                    _desiredOdometry.update(vL_ref * dt, vR_ref * dt);

                    break;
                }
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

            //Update odometry 
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
            // v_l = v - h * ω
            // v_r = v + h * ω
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
