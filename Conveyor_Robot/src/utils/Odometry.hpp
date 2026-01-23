#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Devices/Encoder.hpp"

class Odometry
{
public:
    struct Pose2D
    {
        float x;
        float y;
        float heading;
    };

    void init(const Pose2D &pose);

    void update(Encoder &left, Encoder &right);
    
    Pose2D getPose() const { return _pose; }
    float distanceTravelled() {return _distanceTravelled; };

private:
    Pose2D _pose{0.0f, 0.0f, 0.0f};

    float _prevLeftDistance = 0;
    float _prevRightDistance = 0;
    float _distanceTravelled = 0;

};

#endif
