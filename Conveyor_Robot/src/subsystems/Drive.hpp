#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "Subsystem.h"
#include "Devices/Encoder.hpp"
#include "utils/Odometry.hpp"
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
              const int frontLeftDistPin,
              const int backLeftDistPin,
              const int frontDistPin,
              const int backDistPin,
              const float lKP = DRIVE_L_KP,
              const float lKI = DRIVE_L_KI,
              const float lKD = DRIVE_L_KD,
              const float rKP = DRIVE_R_KP,
              const float rKI = DRIVE_R_KI,
              const float rKD = DRIVE_R_KD);

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

        Encoder _leftEncoder;
        Encoder _rightEncoder;

        MotorController _motorController;

        SharpGP2Y0A51 _frontLeftWallSensor;
        SharpGP2Y0A51 _backLeftWallSensor;
        SharpGP2Y0A51 _frontWallSensor;
        SharpGP2Y0A51 _backWallSensor;

        Odometry _odometry;



};

#endif