/**
 * @file DriveArc.hpp
 * @brief `AutoStep` that drives the robot along a circular arc.
 *
 * Two finish modes are supported:
 *   - **Angle-based**: step ends when the accumulated heading change reaches `targetAngle` (radians).
 *   - **Distance-based**: step ends when odometry distance reaches `distance`.
 */
#ifndef DRIVEARC_STEP_H
#define DRIVEARC_STEP_H

#include "Autonomous/AutoStep.h"
#include "subsystems/Drive.hpp"

/**
 * @brief Commands the drive to follow a radius-constrained arc until an angle or distance target is met.
 */
class DriveArc : public AutoStep{
public:

    /**
     * @brief Construct in angle-based mode (CCW arc).
     * @param drive         Drive subsystem reference.
     * @param targetAngle   Heading change to achieve (radians, positive = CCW).
     * @param targetVelocity Drive speed along the arc (rad/s).
     * @param radius        Arc radius (m).
     */
    DriveArc(Drive& drive,
                float targetAngle,
                float targetVelocity,
                float radius)
        : _drive(drive),
          _target(targetAngle),
          _velocity(targetVelocity),
          _radius(radius),
          _distanceBased(false) {}

    /**
     * @brief Construct in distance-based mode.
     * @param drive          Drive subsystem reference.
     * @param distance       Arc length to travel (m).
     * @param targetVelocity Drive speed (m/s).
     * @param radius         Arc radius (m).
     * @param isDistanceBased Must be `true` to select distance mode.
     */
    DriveArc(Drive &drive,
             float distance,
             float targetVelocity,
             float radius,
             bool isDistanceBased)
        : _drive(drive),
          _target(distance),
          _velocity(targetVelocity),
          _radius(radius),
          _distanceBased(isDistanceBased) {}

    /** @brief Deferred-configure constructor — call `configure()` before use. */
    DriveArc(Drive &drive)
        : _drive(drive){}

    DriveArc();

    void start() override
    {
        _startAngle = _drive.getAccumulatedHeading(); // or encoder average
        _startDistance = _drive.getDistance();
        if(_distanceBased){
            _drive.followRadiusAtVelocity(_velocity, _radius);
            return;
        }
        _drive.followRadiusCCW(_velocity, _radius);
    }

    void update() override
    {
        
    }

    bool isFinished() const override
    {
        // Serial.print(_drive.getAccumulatedHeading() - _startAngle);
        // Serial.print("  ");
        // Serial.println(_target);
        if(_distanceBased){
            // Serial.print("Drive Arc, Delta distance ");
            // Serial.println((_drive.getDistance() - _startDistance));
            return (_drive.getDistance() - _startDistance) >= _target;
        }
        // Serial.print("Drive Arc, Delta angle ");
        // Serial.println(_drive.getAccumulatedHeading() - _startAngle);
        return (_drive.getAccumulatedHeading() - _startAngle) >= _target;
    }

    void end(){
        _drive.setSpeed(0);
    }

    /**
     * @brief Re-configure the arc step for reuse.
     * @param target         Angle (rad) or distance (m) depending on `distanceBased`.
     * @param targetVelocity Drive speed (m/s).
     * @param radius         Arc radius (m).
     * @param direction      `true` = CCW, `false` = CW (currently unused by Drive).
     * @param distanceBased  `true` = distance mode, `false` = angle mode.
     */
    void configure(float target, float targetVelocity, float radius, bool direction, bool distanceBased = false){
        _target = target;
        _velocity = targetVelocity;
        _radius = radius;
        _direction = direction;
        _distanceBased = false;
    }

private:
    Drive &_drive;             ///< Drive subsystem reference
    float _target = 0;         ///< Target angle (rad) or distance (m)
    float _velocity = 0;       ///< Arc travel speed (m/s)
    float _radius = 10;        ///< Arc radius (m)
    bool _direction = 0;       ///< Arc direction (true = CCW)
    float _startAngle = 0;     ///< Accumulated heading at step start (rad)
    bool _distanceBased = false; ///< True = finish on distance, false = finish on angle
    float _startDistance = 0;  ///< Odometry distance at step start (m)
};


#endif