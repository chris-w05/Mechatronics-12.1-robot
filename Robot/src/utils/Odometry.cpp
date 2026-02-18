#include "Odometry.hpp"
#include "Config.hpp"


void Odometry::init(const Pose2D &pose)
{
    _pose = pose;
    _prevLeftDistance = 0;
    _prevRightDistance = 0;
}

void Odometry::update(EncoderWrapper &left, EncoderWrapper &right)
{
    float leftDist = left.read() * PI * DRIVETRAIN_WHEEL_DIAMETER / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV);
    float rightDist = right.read() * PI * DRIVETRAIN_WHEEL_DIAMETER / (DRIVETRAIN_MOTOR_RATIO * TICKS_PER_REV);

    float dLeft = leftDist - _prevLeftDistance;
    float dRight = rightDist - _prevRightDistance;

    _prevLeftDistance = leftDist;
    _prevRightDistance = rightDist;

    float dCenter = (dLeft + dRight) * 0.5f;
    float dTheta = (dRight - dLeft) / DRIVETRAIN_WIDTH;

    _distanceTravelled += dCenter;
    _accumulated_heading += dTheta;
    _pose.heading = _accumulated_heading;
    
    while (_pose.heading > M_PI)
        _pose.heading -= 2.0f * M_PI;
    while (_pose.heading < -M_PI)
        _pose.heading += 2.0f * M_PI;

    _pose.x += dCenter * cos(_pose.heading);
    _pose.y += dCenter * sin(_pose.heading);
}
