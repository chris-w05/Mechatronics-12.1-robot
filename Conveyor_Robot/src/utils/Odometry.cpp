#include "Odometry.hpp"
#include "Config.hpp"

void Odometry::init(const Pose2D &pose)
{
    _pose = pose;
    _prevLeftDistance = 0;
    _prevRightDistance = 0;
}

void Odometry::update(Encoder &left, Encoder &right)
{
    float leftDist = left.getCount() * PI * DRIVETRAIN_WHEEL_DIAMETER / TICKS_PER_REV;
    float rightDist = right.getCount() * PI * DRIVETRAIN_WHEEL_DIAMETER / TICKS_PER_REV;

    float dLeft = leftDist - _prevLeftDistance;
    float dRight = rightDist - _prevRightDistance;

    _prevLeftDistance = leftDist;
    _prevRightDistance = rightDist;

    float dCenter = (dLeft + dRight) * 0.5f;
    float dTheta = (dRight - dLeft) / DRIVETRAIN_WIDTH;

    _distanceTravelled += dCenter;
    _pose.heading += dTheta;

    while (_pose.heading > M_PI)
        _pose.heading -= 2.0f * M_PI;
    while (_pose.heading < -M_PI)
        _pose.heading += 2.0f * M_PI;

    _pose.x += dCenter * cos(_pose.heading);
    _pose.y += dCenter * sin(_pose.heading);
}
