#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Devices/EncoderWrapper.hpp"

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

    void update(EncoderWrapper &left, EncoderWrapper &right);
    
    Pose2D getPose() const { return _pose; }
    float distanceTravelled() {return _distanceTravelled; };

    float getAccumulatedHeading() const {return _accumulated_heading;};

private:
    Pose2D _pose{0.0f, 0.0f, 0.0f};

    float _accumulated_heading = 0;

    float _prevLeftDistance = 0;
    float _prevRightDistance = 0;
    float _distanceTravelled = 0;

};

#endif
