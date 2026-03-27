/**
 * @file Odometry.hpp
 * @brief Differential-drive pose tracker using wheel-encoder odometry.
 *
 * Integrates incremental left/right wheel displacements to maintain a 2-D
 * pose estimate (x, y, heading) and a cumulative heading for multi-turn
 * arc calculations.  All positions are in inches; angles in radians.
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Devices/EncoderWrapper.hpp"

/**
 * @brief Tracks robot pose by integrating encoder wheel displacements.
 *
 * Call `init()` once with the robot's starting pose, then call one of the
 * `update()` overloads every control cycle to integrate wheel travel into
 * a 2-D pose (x, y, heading in radians).
 *
 * Coordinate convention:
 * - +x  points in the direction the robot faces at heading = 0.
 * - +y  points 90° counter-clockwise from +x.
 * - Heading increases counter-clockwise (standard math convention).
 */
class Odometry
{
public:
    /**
     * @brief 2-D robot pose.
     */
    struct Pose2D
    {
        float x;       ///< X position (inches)
        float y;       ///< Y position (inches)
        float heading; ///< Heading angle (radians, wrapped to [-π, π])
    };

    /**
     * @brief Initialise odometry with a known starting pose.
     * @param pose  Initial x, y, and heading (inches, radians).
     */
    void init(const Pose2D &pose);

    /**
     * @brief Update pose from live encoder objects.
     *
     * Reads absolute tick counts from both encoders, computes the change
     * since the last call, and integrates into the pose estimate.
     * @param left   Left-wheel encoder wrapper.
     * @param right  Right-wheel encoder wrapper.
     */
    void update(EncoderWrapper &left, EncoderWrapper &right);

    /**
     * @brief Update pose from incremental wheel displacements.
     *
     * Use this overload when the caller has already computed Δdistance for
     * each wheel (e.g. from a desired-velocity integration for the reference
     * trajectory).
     * @param dLeftPosition   Change in left-wheel position since last call (inches).
     * @param dRightPosition  Change in right-wheel position since last call (inches).
     */
    void update(float dLeftPosition, float dRightPosition);

    /**
     * @brief Return the current estimated pose.
     * @return Pose2D containing x (in), y (in), and heading (rad).
     */
    Pose2D getPose() const { return _pose; }

    /**
     * @brief Return the total arc-length travelled since `init()` (path-dependent).
     * @return Cumulative distance (inches).
     */
    float distanceTravelled() { return _distanceTravelled; };

    /**
     * @brief Return the robot's total heading change since `init()` (path-independent).
     *
     * Unlike `getPose().heading` this value is NOT wrapped to [-π, π]; it
     * continuously accumulates across multiple full rotations and is useful
     * for tracking the angle swept during arc manoeuvres.
     * @return Accumulated heading (radians).
     */
    float getAccumulatedHeading() const { return _accumulated_heading; };

private:
    Pose2D _pose{0.0f, 0.0f, 0.0f}; ///< Current estimated pose

    float _accumulated_heading = 0;  ///< Total heading change since init (radians, unwrapped)

    float _prevLeftDistance  = 0;    ///< Last recorded left-wheel absolute position (inches)
    float _prevRightDistance = 0;    ///< Last recorded right-wheel absolute position (inches)
    float _distanceTravelled = 0;    ///< Cumulative path length since init (inches)

};

#endif
