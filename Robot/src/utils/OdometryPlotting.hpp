/**
 * @file OdometryPlotting.hpp
 * @brief Template helper functions for emitting JSON odometry telemetry over a serial port.
 *
 * Three overloads of `emitTelemetryJSON()` are provided, each adding more fields
 * to the JSON object:
 * 1. Actual pose + actual wheel velocities and positions.
 * 2. Adds desired (reference) wheel positions.
 * 3. Adds desired pose, desired wheel velocities, and 2-D pose error.
 *
 * The output is a single-line JSON object terminated by a newline, compatible with
 * the Teleplot VS Code extension and other serial JSON consumers.
 *
 * @note These are template functions parameterised on an odometry type.  The type
 *       must expose a `getPose()` method returning an object with public `x`, `y`,
 *       and `heading` fields.
 */
#pragma once
#include <Arduino.h>

/**
 * @brief Wrap an angle to the range [-pi, pi].
 * @param a  Input angle (radians).
 * @return   Equivalent angle in (-π, π].
 */
inline __attribute__((unused)) float wrapAnglePi(float a)
{
    while (a > PI)
        a -= 2.0f * PI;
    while (a < -PI)
        a += 2.0f * PI;
    return a;
}

/*
Expected Odometry API:
    Pose2D getPose() const;
    where Pose2D has:
        float x;
        float y;
        float heading;
*/

// =====================================================
/// @name Telemetry emitters
/// @{
// =====================================================

/**
 * @brief Emit a JSON object with the robot's actual pose and wheel state.
 *
 * JSON fields:
 * | Key  | Description                          | Units  |
 * |------|--------------------------------------|--------|
 * | t    | Timestamp from millis()              | ms     |
 * | x    | Actual X position                    | inches |
 * | y    | Actual Y position                    | inches |
 * | th   | Actual heading                       | rad    |
 * | vl   | Actual left-wheel velocity           | in/s   |
 * | vr   | Actual right-wheel velocity          | in/s   |
 * | pl   | Actual left-wheel position           | inches |
 * | pr   | Actual right-wheel position          | inches |
 *
 * @tparam OdomT       Odometry type (must provide `getPose()`).
 * @param serial        Serial port to write to.
 * @param actualOdom    Current (measured) odometry object.
 * @param vLeftActual   Left-wheel velocity (in/s).
 * @param vRightActual  Right-wheel velocity (in/s).
 * @param pLeftActual   Left-wheel position (in).
 * @param pRightActual  Right-wheel position (in).
 */
template <typename OdomT>
void emitTelemetryJSON(
    HardwareSerial &serial,
    const OdomT &actualOdom,
    float vLeftActual,
    float vRightActual,
    float pLeftActual,
    float pRightActual)
{
    const unsigned long t = millis();
    const auto pose = actualOdom.getPose();

    serial.print('{');

    serial.print("\"t\":");
    serial.print(t);

    serial.print(",\"x\":");
    serial.print(pose.x, 6);

    serial.print(",\"y\":");
    serial.print(pose.y, 6);

    serial.print(",\"th\":");
    serial.print(pose.heading, 6);

    serial.print(",\"vl\":");
    serial.print(vLeftActual, 6);

    serial.print(",\"vr\":");
    serial.print(vRightActual, 6);

    serial.print(",\"pl\":");
    serial.print(pLeftActual, 6);

    serial.print(",\"pr\":");
    serial.print(pRightActual, 6);

    serial.println('}');
}

/**
 * @brief Emit a JSON object with actual and desired wheel positions.
 *
 * Extends the basic overload with reference (desired) wheel positions so that
 * tracking error can be computed on the receiving side.
 *
 * Additional fields beyond the basic overload:
 * | Key    | Description                  | Units  |
 * |--------|------------------------------|--------|
 * | pl_ref | Desired left-wheel position  | inches |
 * | pr_ref | Desired right-wheel position | inches |
 *
 * @tparam OdomT         Odometry type (must provide `getPose()`).
 * @param serial          Serial port to write to.
 * @param actualOdom      Current (measured) odometry object.
 * @param vLeftActual     Actual left-wheel velocity (in/s).
 * @param vRightActual    Actual right-wheel velocity (in/s).
 * @param pLeftActual     Actual left-wheel position (in).
 * @param pRightActual    Actual right-wheel position (in).
 * @param pLeftDesired    Reference left-wheel position (in).
 * @param pRightDesired   Reference right-wheel position (in).
 */
template <typename OdomT>
void emitTelemetryJSON(
    HardwareSerial &serial,
    const OdomT &actualOdom,
    float vLeftActual,
    float vRightActual,
    float pLeftActual,
    float pRightActual,
    float pLeftDesired,
    float pRightDesired)
{
    const unsigned long t = millis();
    const auto pose = actualOdom.getPose();

    serial.print('{');

    serial.print("\"t\":");
    serial.print(t);

    serial.print(",\"x\":");
    serial.print(pose.x, 6);

    serial.print(",\"y\":");
    serial.print(pose.y, 6);

    serial.print(",\"th\":");
    serial.print(pose.heading, 6);

    serial.print(",\"vl\":");
    serial.print(vLeftActual, 6);

    serial.print(",\"vr\":");
    serial.print(vRightActual, 6);

    serial.print(",\"pl\":");
    serial.print(pLeftActual, 6);

    serial.print(",\"pr\":");
    serial.print(pRightActual, 6);

    serial.print(",\"pl_ref\":");
    serial.print(pLeftDesired, 6);

    serial.print(",\"pr_ref\":");
    serial.print(pRightDesired, 6);

    serial.println('}');
}

/**
 * @brief Emit a JSON object with full actual + desired pose and 2-D tracking error.
 *  The most complete telemetry overload.  
 * In addition to the fields in the two\n * simpler overloads, it adds the desired pose, desired wheel velocities, and\n * the signed 2-D pose error (ex, ey, eth) so that trajectory tracking can be
 *  visualised in real time.
 *  @tparam OdomT          Odometry type (must provide `getPose()`)
 *  @param serial           Serial port to write to.
 *  @param actualOdom       Current (measured) odometry object.
 *  @param desiredOdom      Reference (desired) odometry object.
 *  @param vLeftActual      Actual left-wheel velocity (in/s).
 *  @param vRightActual     Actual right-wheel velocity (in/s).
 *  @param vLeftDesired     Reference left-wheel velocity (in/s).
 *  @param vRightDesired    Reference right-wheel velocity (in/s).
 *  @param pLeftActual      Actual left-wheel position (in).
 *  @param pRightActual     Actual right-wheel position (in).
 *  @param pLeftDesired     Reference left-wheel position (in).
 *  @param pRightDesired    Reference right-wheel position (in).
 *  */
template <typename OdomT> void emitTelemetryJSON(
    HardwareSerial &serial,
        const OdomT &actualOdom,
        const OdomT &desiredOdom,
            float vLeftActual,
            float vRightActual,
            float vLeftDesired,
            float vRightDesired,    
            float pLeftActual,   
            float pRightActual,    
            float pLeftDesired,    
            float pRightDesired)
{
    const unsigned long t = millis();

    const auto actualPose = actualOdom.getPose();
    const auto desiredPose = desiredOdom.getPose();

    const float x = actualPose.x;
    const float y = actualPose.y;
    const float th = actualPose.heading;

    const float xd = desiredPose.x;
    const float yd = desiredPose.y;
    const float thd = desiredPose.heading;

    serial.print('{');

    serial.print("\"t\":");
    serial.print(t);

    serial.print(",\"x\":");
    serial.print(x, 6);

    serial.print(",\"y\":");
    serial.print(y, 6);

    serial.print(",\"th\":");
    serial.print(th, 6);

    serial.print(",\"vl\":");
    serial.print(vLeftActual, 6);

    serial.print(",\"vr\":");
    serial.print(vRightActual, 6);

    serial.print(",\"pl\":");
    serial.print(pLeftActual, 6);

    serial.print(",\"pr\":");
    serial.print(pRightActual, 6);

    serial.print(",\"xd\":");
    serial.print(xd, 6);

    serial.print(",\"yd\":");
    serial.print(yd, 6);

    serial.print(",\"thd\":");
    serial.print(thd, 6);

    serial.print(",\"vl_ref\":");
    serial.print(vLeftDesired, 6);

    serial.print(",\"vr_ref\":");
    serial.print(vRightDesired, 6);

    serial.print(",\"pl_ref\":");
    serial.print(pLeftDesired, 6);

    serial.print(",\"pr_ref\":");
    serial.print(pRightDesired, 6);

    serial.print(",\"ex\":");
    serial.print(xd - x, 6);

    serial.print(",\"ey\":");
    serial.print(yd - y, 6);

    serial.print(",\"eth\":");
    serial.print(wrapAnglePi(thd - th), 6);

    serial.println('}');
}

// =====================================================
// CSV: actual only
// Format:
// P,t,x,y,th,vl,vr,pl,pr
// =====================================================
template <typename OdomT>
void emitTelemetryCSV(
    HardwareSerial &serial,
    const OdomT &actualOdom,
    float vLeftActual,
    float vRightActual,
    float pLeftActual,
    float pRightActual)
{
    const auto pose = actualOdom.getPose();

    serial.print("P,");
    serial.print(millis());

    serial.print(",");
    serial.print(pose.x, 6);
    serial.print(",");
    serial.print(pose.y, 6);
    serial.print(",");
    serial.print(pose.heading, 6);

    serial.print(",");
    serial.print(vLeftActual, 6);
    serial.print(",");
    serial.print(vRightActual, 6);

    serial.print(",");
    serial.print(pLeftActual, 6);
    serial.print(",");
    serial.println(pRightActual, 6);
}

// =====================================================
// CSV: actual + desired wheel positions only
// Format:
// P,t,x,y,th,vl,vr,pl,pr,pl_ref,pr_ref
// =====================================================
template <typename OdomT>
void emitTelemetryCSV(
    HardwareSerial &serial,
    const OdomT &actualOdom,
    float vLeftActual,
    float vRightActual,
    float pLeftActual,
    float pRightActual,
    float pLeftDesired,
    float pRightDesired)
{
    const auto pose = actualOdom.getPose();

    serial.print("P,");
    serial.print(millis());

    serial.print(",");
    serial.print(pose.x, 6);
    serial.print(",");
    serial.print(pose.y, 6);
    serial.print(",");
    serial.print(pose.heading, 6);

    serial.print(",");
    serial.print(vLeftActual, 6);
    serial.print(",");
    serial.print(vRightActual, 6);

    serial.print(",");
    serial.print(pLeftActual, 6);
    serial.print(",");
    serial.print(pRightActual, 6);

    serial.print(",");
    serial.print(pLeftDesired, 6);
    serial.print(",");
    serial.println(pRightDesired, 6);
}

// =====================================================
// CSV: actual + desired odometry + desired wheel data
// Format:
// P,t,x,y,th,xd,yd,thd,vl,vr,vl_ref,vr_ref,pl,pr,pl_ref,pr_ref
// =====================================================
template <typename OdomT>
void emitTelemetryCSV(
    HardwareSerial &serial,
    const OdomT &actualOdom,
    const OdomT &desiredOdom,
    float vLeftActual,
    float vRightActual,
    float vLeftDesired,
    float vRightDesired,
    float pLeftActual,
    float pRightActual,
    float pLeftDesired,
    float pRightDesired)
{
    const auto actualPose = actualOdom.getPose();
    const auto desiredPose = desiredOdom.getPose();

    serial.print("P,");
    serial.print(millis());

    serial.print(",");
    serial.print(actualPose.x, 6);
    serial.print(",");
    serial.print(actualPose.y, 6);
    serial.print(",");
    serial.print(actualPose.heading, 6);

    serial.print(",");
    serial.print(desiredPose.x, 6);
    serial.print(",");
    serial.print(desiredPose.y, 6);
    serial.print(",");
    serial.print(desiredPose.heading, 6);

    serial.print(",");
    serial.print(vLeftActual, 6);
    serial.print(",");
    serial.print(vRightActual, 6);

    serial.print(",");
    serial.print(vLeftDesired, 6);
    serial.print(",");
    serial.print(vRightDesired, 6);

    serial.print(",");
    serial.print(pLeftActual, 6);
    serial.print(",");
    serial.print(pRightActual, 6);

    serial.print(",");
    serial.print(pLeftDesired, 6);
    serial.print(",");
    serial.println(pRightDesired, 6);
}