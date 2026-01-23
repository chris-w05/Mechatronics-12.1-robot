#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/Encoder.hpp"
#include "utils/PID.hpp"
#include "Devices/MotorController.hpp"
#include "Config.hpp"

class Gripper : public Subsystem
{
public:
    Gripper(const int servoPin):
        clampServo(servoPin)
        {}

    enum Position{
        CLOSED,
        OPEN
    };

    void init() override
    {
        clampServo.init();
    }

    void update() override
    {
        switch(_targetPosition){
        case OPEN:
            _targetAngle = GRIPPER_OPEN_ANGLE;
            break;
        case CLOSED:
            _targetAngle = GRIPPER_CLOSED_ANGLE;
        }

        clampServo.setAngle(_targetAngle);
        clampServo.update();


    }

    void setPosition(Position pos)
    {
        _targetPosition = pos;
    }

    void stop() override
    {
    }
    // Needs to be updated if using PID

private:
    Position _targetPosition = OPEN;
    int _targetAngle  = 0;

    ServoControl clampServo; 
};