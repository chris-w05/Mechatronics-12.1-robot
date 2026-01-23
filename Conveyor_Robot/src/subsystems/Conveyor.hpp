#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/MotorController.hpp"
#include "Config.hpp"
#include "Devices/Encoder.hpp"

class Conveyor : public Subsystem
{
public:
    Conveyor() :
        _motorController(
            ARM_ROT_KP, ARM_ROT_KI, ARM_ROT_KD, false,
            ARM_EXT_KP, ARM_EXT_KI, ARM_EXT_KD, false,
            true, true)
    {}

    void init() override
    {
        
    }

    void setPreset(double power)
    {
        _targetPower = power;
    }

    void update() override
    {
        _motorController.setPower(_targetPower, 0);
    }



    void stop() override
    {
    }



private:
    // M1 -> Rotation
    // M2 -> translation
    MotorController _motorController;
    int _targetVelocity = 90;
    int _targetPower = 0;

};