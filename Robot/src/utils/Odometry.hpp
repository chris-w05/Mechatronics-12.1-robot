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

    /**
     * Update the odometry. This will integrate the robot's velocity and heading to track its position in relative coordinates
     */
    void update(EncoderWrapper &left, EncoderWrapper &right);
    
    /**
     * Get the x, y, theta of the robot 
     */
    Pose2D getPose() const { return _pose; }

    /**
     * Get the total distance travelled by the robot, relative to the starting position. 
     * This is path dependent
     */
    float distanceTravelled() {return _distanceTravelled; };

    /** 
     * Get the total number of rotation of the robot, relative to starting.
     * This is path independent
     */
    float getAccumulatedHeading() const {return _accumulated_heading;};

private:
    Pose2D _pose{0.0f, 0.0f, 0.0f};

    float _accumulated_heading = 0;

    float _prevLeftDistance = 0;
    float _prevRightDistance = 0;
    float _distanceTravelled = 0;

};

#endif
