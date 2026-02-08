#ifndef DRIVE_HPP
#define DRIVE_HPP

#include <Arduino.h>
#include "Subsystem.h"
#include "utils/Odometry.hpp"
#include "Devices/EncoderWrapper.hpp"
#include "Devices/MotorController.hpp"
#include "Devices/DistanceSensor.hpp"

class Drive : public Subsystem {
    public:
        Drive(const int left_enc_a,
              const int left_enc_b,
              const int right_enc_a,
              const int right_enc_b,
              const int left_mtr_pwm,
              const int left_mtr_dir,
              const int right_mtr_pwm,
              const int right_mtr_dir,
              const int distPin);

        void init() override;
        void update() override;
        void stop() override;

        Odometry::Pose2D getPose();
        void setSpeed(int16_t speed);
        float getDistance() {return _odometry.distanceTravelled();};
        void followRadiusClockwise(int16_t speed, float radius);
        void followRadiusCCW(int16_t speed, float radius);

    private:
        int16_t _speedL = 0;
        int16_t _speedR = 0;

        EncoderWrapper _leftEncoder;
        EncoderWrapper _rightEncoder;

        MotorController _motorController;

        SharpGP2Y0A51 distSensor;

        Odometry _odometry;



};

#endif