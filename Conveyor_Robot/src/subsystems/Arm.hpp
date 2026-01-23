#pragma once

#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/MotorController.hpp"
#include "Config.hpp"
#include "Devices/Encoder.hpp"

class Arm : public Subsystem{
    public:
        Arm(
            const int rot_enc_a,
            const int rot_enc_b,
            const int ext_enc_a,
            const int ext_enc_b
            ) : _extEncoder(ARM_EXT_ENCODER_A, ARM_EXT_ENCODER_B),
                _rotEncoder(ARM_ROT_ENCODER_A, ARM_ROT_ENCODER_B),
                _motorControler(
                    ARM_ROT_KP, ARM_ROT_KI, ARM_ROT_KD, false,
                    ARM_EXT_KP, ARM_EXT_KI, ARM_EXT_KD, false,
                    true, true)
        {
        }

        void init() override {
            _motorControler.init();
        }

        void setPreset(ArmPresetId pid)
        {
            if (pid < 0 || pid >= PRESET_COUNT)
                return;
            const ArmPreset &p = ARM_PRESETS[pid];
            setPosition(p.angle, p.extension);
        }

        void update() override {
            _motorControler.setTarget(_targetAngle, _targetExtension);
            
            _extEncoder.update();
            _rotEncoder.update();

            _currentExtension = _extEncoder.getPosition() * ARM_EXT_RATIO;
            _currentRotation = _rotEncoder.getPosition() * ARM_ROT_RATIO;

            _motorControler.update(_currentRotation, _currentExtension);
        }

        void setPosition(int angle, int extension){
            _targetAngle = angle;
            _targetExtension = extension;
        }

        void stop() override{

        }

        //Needs to be updated if using PID
        double getRotation( ){
            return _currentRotation;
        }

        double getExtension(){
            return _currentExtension;
        }

    private:
        //M1 -> Rotation
        //M2 -> translation
        
        
        Encoder _extEncoder;
        Encoder _rotEncoder;
        MotorController _motorControler;
        int _targetAngle = 90;
        int _targetExtension = 90;
        double _currentExtension = 0;
        double _currentRotation = 0;
};