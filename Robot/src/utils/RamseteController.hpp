/**
 * @file RamseteController.hpp
 * @brief Ramsete nonlinear trajectory-tracking controller for a differential-drive robot.
 *
 * Implements the RAMSETE algorithm (Banni, Brandin & Sciavicco, 1996).  Given a
 * reference pose + feedforward wheel velocities at each time step, the controller
 * outputs corrected linear (v) and angular (w) chassis velocities that drive the
 * robot back onto the reference trajectory while avoiding oscillation.
 *
 * Typical usage inside the Drive `update()` loop:
 * @code
 * auto cmd = _ramsete.step(refPose, vRef, wRef, actualPose);
 * float vL = cmd.v - cmd.w * DRIVETRAIN_WIDTH / 2.0f;
 * float vR = cmd.v + cmd.w * DRIVETRAIN_WIDTH / 2.0f;
 * @endcode
 */
#pragma once

#include <Arduino.h>
#include "utils/Odometry.hpp"

/**
 * Ramsete nonlinear tracking controller for differential-drive robots.
 *
 * Given a reference pose and feedforward chassis velocities, it produces
 * corrected linear and angular velocity commands that drive the robot
 * toward a reference trajectory while compensating for pose error.
 *
 * Reference: Ramsete — A non-linear time-varying feedback controller for
 * mobile robots on a sampled-data basis (Banni, Brandin, Sciavicco, 1996).
 */
class RamseteController {
public:

    struct Cmd {
        float v; ///< Corrected linear velocity (in/s)
        float w; ///< Corrected angular velocity (rad/s)
    };

    /**
     * @param b    Positive control gain. Larger values give more aggressive
     *             cross-track correction.
     * @param zeta Damping ratio in (0, 1). Values closer to 1 are more
     *             overdamped and avoid oscillation.
     */
    RamseteController(float b = 0.04f, float zeta = 0.1f)
        : _b(b), _zeta(zeta) {}

    void enable()          { _enabled = true;  }
    void disable()         { _enabled = false; }
    bool isEnabled() const { return _enabled;  }

    /**
     * Compute one control step.
     *
     * @param ref   Desired (reference) pose at this instant
     * @param vRef  Feedforward linear  velocity (in/s)
     * @param wRef  Feedforward angular velocity (rad/s)
     * @param cur   Current estimated robot pose
     * @return      Corrected chassis velocity commands
     */
    Cmd step(const Odometry::Pose2D &ref,
             float vRef, float wRef,
             const Odometry::Pose2D &cur) const
    {
        // Compute pose error rotated into the robot's local frame
        float dx = ref.x - cur.x;
        float dy = ref.y - cur.y;
        float c  = cosf(cur.heading);
        float s  = sinf(cur.heading);

        float ex     =  c * dx + s * dy;
        float ey     = -s * dx + c * dy;
        float etheta = wrapPi(ref.heading - cur.heading);

        float k = 2.0f * _zeta * sqrtf(wRef * wRef + _b * vRef * vRef);

        return {
            vRef * cosf(etheta) + k * ex,
            wRef + k * etheta + _b * vRef * sinc(etheta) * ey
        };
    }

private:
    float _b;
    float _zeta;
    bool  _enabled = true;

    static float wrapPi(float a)
    {
        while (a >  M_PI) a -= 2.0f * M_PI;
        while (a < -M_PI) a += 2.0f * M_PI;
        return a;
    }

    // sinc(x) = sin(x)/x, with a Taylor series for small x to avoid division by zero
    static float sinc(float x)
    {
        return fabsf(x) < 1e-4f ? 1.0f - x * x / 6.0f : sinf(x) / x;
    }
};
