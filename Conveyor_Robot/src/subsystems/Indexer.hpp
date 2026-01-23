#include "Subsystem.h"
#include "Devices/ServoControl.hpp"
#include "Devices/Encoder.hpp"
#include "utils/PID.hpp"
#include "Devices/MotorController.hpp"
#include "Config.hpp"

class Indexer : public Subsystem
{
public:
    Indexer(const int servoPin):
    indexerServo(servoPin)
    {}

    enum Position
    {
        LEFT,
        MIDDLE,
        RIGHT
    };

    void init() override
    {
        indexerServo.init();
    }

    void update() override
    {
        switch (_targetPosition)
        {
        case LEFT:
            _targetAngle = INDEXER_LEFT_ANGLE;
            break;
        case MIDDLE:
            _targetAngle = INDEXER_MIDDLE_ANGLE;
            break;
        case RIGHT:
            _targetAngle = INDEXER_RIGHT_ANGLE;
        }

        indexerServo.setAngle(_targetAngle);
        indexerServo.update();
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
    Position _targetPosition = MIDDLE;
    int _targetAngle = 0;

    ServoControl indexerServo;
};