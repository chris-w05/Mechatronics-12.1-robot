// Lab11_PID_DualMotor.ino
// Dual-motor PID position control
// Motor library: L298NMotorDriverMega
// Encoder library: Encoder

#include <Encoder.h>
#include <L298NMotorDriverMega.h>

// ── Hardware ────────────────────────────────────────────────────────────────
Encoder leftEncoder(18, 19);   // Left  motor encoder
Encoder rightEncoder(20, 21);  // Right motor encoder

L298NMotorDriverMega motorDriver(
  6, 31, 33,   // Left  motor  (PWM, DIR_A, DIR_B)
  7, 35, 37);  // Right motor  (PWM, DIR_A, DIR_B)

// ── Robot / encoder constants ────────────────────────────────────────────────
const double GearRatio        = 50.0;
const double EncoderCountsPerRev = 64.0;

// ── Time ─────────────────────────────────────────────────────────────────────
double t, t_old, t0, deltaT;

// ── Motor 1 (Left) state ─────────────────────────────────────────────────────
double theta1, omega1, omega1f = 0;
double theta1_old = 0;
double theta1_des, omega1_des;
double error1      = 0;
double dErrordt1   = 0;
double integralError1 = 0;
double V1 = 0;

// ── Motor 2 (Right) state ────────────────────────────────────────────────────
double theta2, omega2, omega2f = 0;
double theta2_old = 0;
double theta2_des, omega2_des;
double error2      = 0;
double dErrordt2   = 0;
double integralError2 = 0;
double V2 = 0;

// ── PID / trajectory parameters (received over Serial) ───────────────────────
double Kp, Kd, Ki;
double StepSize;
double alpha = .1;        // low-pass filter weight  (0 < alpha < 1)
double StepDuration;

// Motor command limits (±400 maps to ±10 V for L298N shield)
const int CMD_MAX = 400;
const double V_MAX = 10.0;  // volts (logical max used in scaling)

// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(250000);
  Serial.println("Arduino Ready to Receive Control Parameters");

  motorDriver.init();

  leftEncoder.write(0);
  rightEncoder.write(0);

  // ── Wait for parameters sent as space-delimited string ───────────────────
  // Format: "Kp Kd Ki StepSize alpha StepDuration "
  // (trailing space terminates the last token)
  while (Serial.available() == 0)
    ;

  String inString = "";
  inString   = Serial.readStringUntil(' ');  Kp           = inString.toFloat();
  inString   = Serial.readStringUntil(' ');  Kd           = inString.toFloat();
  inString   = Serial.readStringUntil(' ');  Ki           = inString.toFloat();
  inString   = Serial.readStringUntil(' ');  StepSize     = inString.toFloat();
  inString   = Serial.readStringUntil(' ');  alpha        = inString.toFloat();
  inString   = Serial.readStringUntil(' ');  StepDuration = inString.toFloat();

  t0    = micros() / 1000000.0;
  t_old = 0.0;
}

// ────────────────────────────────────────────────────────────────────────────
void loop() {

  // ── Time ──────────────────────────────────────────────────────────────────
  t      = micros() / 1000000.0 - t0;
  deltaT = t - t_old;

  // Guard against zero deltaT on first pass or timer glitch
  if (deltaT <= 0) return;

  // ── Encoder sensing ───────────────────────────────────────────────────────
  theta1 = leftEncoder.read()  * 2.0 * PI / (GearRatio * EncoderCountsPerRev);
  theta2 = rightEncoder.read() * 2.0 * PI / (GearRatio * EncoderCountsPerRev);

  // ── Raw velocities ────────────────────────────────────────────────────────
  omega1 = (theta1 - theta1_old) / deltaT;
  omega2 = (theta2 - theta2_old) / deltaT;

  // ── Low-pass filter on velocity ───────────────────────────────────────────
  omega1f = alpha * omega1 + (1.0 - alpha) * omega1f;
  omega2f = alpha * omega2 + (1.0 - alpha) * omega2f;

  // ── Desired trajectory (step input) ──────────────────────────────────────
  theta1_des = StepSize;
  theta2_des = StepSize;
  omega1_des = 0.0;
  omega2_des = 0.0;

  // ── PID errors ────────────────────────────────────────────────────────────
  double error1_prev = error1;
  double error2_prev = error2;

  error1 = theta1_des - theta1;
  error2 = theta2_des - theta2;

  dErrordt1 = (error1 - error1_prev) / deltaT;
  dErrordt2 = (error2 - error2_prev) / deltaT;

  integralError1 += error1 * deltaT;
  integralError2 += error2 * deltaT;

  // ── PID control law ───────────────────────────────────────────────────────
  V1 = Kp * error1 + Kd * dErrordt1 + Ki * integralError1;
  V2 = Kp * error2 + Kd * dErrordt2 + Ki * integralError2;

  // ── Clamp voltage commands ────────────────────────────────────────────────
  V1 = constrain(V1, -V_MAX, V_MAX);
  V2 = constrain(V2, -V_MAX, V_MAX);

  // ── Send motor commands  (±400 = full scale for L298NMotorDriverMega) ─────
  int cmd1 = (int)(CMD_MAX * V1 / V_MAX);
  int cmd2 = (int)(CMD_MAX * V2 / V_MAX);
  motorDriver.setSpeeds(cmd1, cmd2);

  // ── Serial data logging ───────────────────────────────────────────────────
  if (t < StepDuration) {
    // // Motor 1 (Left)
    // Serial.print(t, 5);           Serial.print('\t');
    // Serial.print(theta1, 5);      Serial.print('\t');
    // Serial.print(theta1_des, 5);  Serial.print('\t');
    // Serial.print(error1, 5);      Serial.print('\t');
    // Serial.print(dErrordt1, 5);   Serial.print('\t');
    // Serial.print(integralError1, 5); Serial.print('\t');
    // Serial.print(V1, 5);          Serial.print('\t');
    // // Motor 2 (Right)
    // Serial.print(theta2, 5);      Serial.print('\t');
    // Serial.print(theta2_des, 5);  Serial.print('\t');
    // Serial.print(error2, 5);      Serial.print('\t');
    // Serial.print(dErrordt2, 5);   Serial.print('\t');
    // Serial.print(integralError2, 5); Serial.print('\t');
    // Serial.print(V2, 5);
    // Serial.println();

    //Printing averages for drivetrain
    Serial.print(t, 5);                                       Serial.print('\t');
    Serial.print((theta1 + theta2)/2.0, 5);                   Serial.print('\t');
    Serial.print((dErrordt1 + dErrordt2)/2.0, 5);             Serial.print('\t');
    Serial.print((integralError1 + integralError2)/2.0, 5);   Serial.print('\t');
    Serial.print((V1 + V2)/2.0, 5);                           Serial.print('\t');
    Serial.print((theta1_des + theta2_des)/2.0, 5);
    Serial.println();
  } else {
    // Stop motors after StepDuration
    motorDriver.setSpeeds(0, 0);
  }

  // ── Prepare for next iteration ────────────────────────────────────────────
  t_old      = t;
  theta1_old = theta1;
  theta2_old = theta2;
}
