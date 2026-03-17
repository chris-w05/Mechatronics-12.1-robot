#pragma once
#include <Arduino.h>

// Wrap angle to [-pi, pi]
static float wrapAnglePi(float a)
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
// JSON: actual only
// Fields: t, x, y, th, vl, vr, pl, pr
// =====================================================
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

// =====================================================
// JSON: actual + desired wheel positions only
// Fields: t, x, y, th, vl, vr, pl, pr, pl_ref, pr_ref
// =====================================================
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

// =====================================================
// JSON: actual + desired odometry + desired wheel data
// Fields: t, x, y, th, vl, vr, pl, pr,
//         xd, yd, thd, vl_ref, vr_ref, pl_ref, pr_ref,
//         ex, ey, eth
// =====================================================
template <typename OdomT>
void emitTelemetryJSON(
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