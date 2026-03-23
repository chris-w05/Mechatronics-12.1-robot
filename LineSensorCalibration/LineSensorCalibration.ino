/*
  LineSensor Calibration Utility
  QTR-MD-08RC — 8 reflectance sensors
  Pins: 28, 30, 32, 34, 36, 38, 40, 42  (must match Config.hpp LINE_SENSOR_PINS)

  How to use
  ----------
  1. Upload to the Mega and open Serial Monitor at 115200 baud.
  2. Phase 1 — Baseline: place the sensor flat over a typical white field
     surface and press ENTER. The sketch prints raw RC discharge times
     (µs) so you can check all sensors are responding.
  3. Phase 2 — Calibration sweep: when prompted, SLOWLY sweep the sensor
     back and forth across the track (white background + black line) for
     the full 5-second window. The library records per-sensor min/max.
  4. The sketch prints ready-to-paste LINESENSORCALMIN[] / LINESENSORCALMAX[]
     arrays for Config.hpp, plus a per-sensor breakdown table.
  5. Phase 3 — Live verification: calibrated bar graphs and computed
     position are printed continuously so you can confirm detection is
     working before writing calibration values back into the firmware.

  Note: RC sensors give LOWER discharge times over bright (white)
  surfaces and HIGHER times over dark (black line) surfaces.
*/

#include <Arduino.h>
#include <QTRSensors.h>

// ── Configuration ──────────────────────────────────────────────────────────
static const uint8_t  NUM_SENSORS           = 8;
static const uint8_t  SENSOR_PINS[NUM_SENSORS] = {28, 30, 32, 34, 36, 38, 40, 42};
static const uint16_t RC_TIMEOUT_US         = 1000;   // µs — matches LineSensor::init default
static const float    SENSOR_SPACING_CM     = 0.8f;   // cm between adjacent sensors

static const uint32_t CALIBRATION_MS        = 5000;   // sweep duration
static const uint32_t CALIBRATION_RATE_HZ   = 50;     // calibrate() calls per second
// ───────────────────────────────────────────────────────────────────────────

QTRSensors qtr;
uint16_t   sensorValues[NUM_SENSORS];

// ── Helpers ────────────────────────────────────────────────────────────────

// ASCII bar proportional to value in [0, maxVal]
void printBar(uint16_t value, uint16_t maxVal, uint8_t width = 15)
{
    uint8_t filled = (uint32_t)value * width / maxVal;
    Serial.print('[');
    for (uint8_t i = 0; i < width; i++)
        Serial.print(i < filled ? '#' : ' ');
    Serial.print(']');
}

// Block until the user presses Enter (newline)
void waitForEnter()
{
    while (Serial.available()) Serial.read();   // flush old bytes
    while (!Serial.available()) { /* spin */ }
    while (Serial.available())  Serial.read();  // consume newline
}

// Left-pad a uint16 to a given field width
void printPadded(uint16_t v, uint8_t width)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%*u", (int)width, (unsigned)v);
    Serial.print(buf);
}

// ── setup ──────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait for USB CDC */ }

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("   LineSensor Calibration Utility"));
    Serial.println(F("   QTR-MD-08RC  |  8 sensors  |  Mega"));
    Serial.println(F("========================================"));
    Serial.println();

    // ── Init QTR ──
    qtr.setTypeRC();
    qtr.setTimeout(RC_TIMEOUT_US);
    qtr.setSensorPins(SENSOR_PINS, NUM_SENSORS);

    // ── Phase 1: raw baseline ─────────────────────────────────────────────
    Serial.println(F("PHASE 1 — Baseline"));
    Serial.println(F("Place sensor flat over a typical WHITE field surface."));
    Serial.println(F("Press ENTER when ready."));
    waitForEnter();

    qtr.read(sensorValues);
    Serial.println();
    Serial.println(F("Raw discharge times over white (µs)  [lower = brighter]:"));
    Serial.println(F("  Idx | Value | Bar"));
    Serial.println(F("  ----+-------+------------------"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(F("   "));
        Serial.print(i);
        Serial.print(F("  | "));
        printPadded(sensorValues[i], 5);
        Serial.print(F(" | "));
        printBar(sensorValues[i], RC_TIMEOUT_US);
        Serial.println();
    }
    Serial.println();

    // ── Phase 2: calibration sweep ────────────────────────────────────────
    Serial.println(F("PHASE 2 — Calibration Sweep"));
    Serial.println(F("Slowly sweep the sensor back and forth across the track,"));
    Serial.println(F("covering both the WHITE background and the BLACK line."));
    Serial.print  (F("You will have "));
    Serial.print  (CALIBRATION_MS / 1000);
    Serial.println(F(" seconds once calibration begins."));
    Serial.println(F("Press ENTER to start."));
    waitForEnter();

    Serial.println(F(">>> MOVE SENSOR NOW <<<"));
    uint32_t start    = millis();
    uint32_t lastTick = 0;
    while (millis() - start < CALIBRATION_MS) {
        qtr.calibrate();
        uint32_t elapsed = millis() - start;
        if (elapsed - lastTick >= 500) {
            uint32_t remaining = (CALIBRATION_MS - elapsed + 999) / 1000;
            Serial.print(F("  "));
            Serial.print(remaining);
            Serial.println(F("s remaining..."));
            lastTick = elapsed;
        }
        delay(1000 / CALIBRATION_RATE_HZ);
    }
    Serial.println(F("Calibration complete!"));
    Serial.println();

    // ── Report ────────────────────────────────────────────────────────────
    Serial.println(F("========================================"));
    Serial.println(F("  CALIBRATION RESULTS"));
    Serial.println(F("========================================"));
    Serial.println();

    // Ready-to-paste arrays
    Serial.println(F("Copy the two lines below into Config.hpp:"));
    Serial.println();
    Serial.print  (F("const uint16_t LINESENSORCALMIN[8] = {"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        if (i < NUM_SENSORS - 1) Serial.print(F(", "));
    }
    Serial.println(F("};"));

    Serial.print  (F("const uint16_t LINESENSORCALMAX[8] = {"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        if (i < NUM_SENSORS - 1) Serial.print(F(", "));
    }
    Serial.println(F("};"));
    Serial.println();

    // Per-sensor breakdown table
    Serial.println(F("Per-sensor breakdown:"));
    Serial.println(F("  Idx |  Min  |  Max  | Range | OK?"));
    Serial.println(F("  ----+-------+-------+-------+-----"));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint16_t lo    = qtr.calibrationOn.minimum[i];
        uint16_t hi    = qtr.calibrationOn.maximum[i];
        uint16_t range = hi - lo;
        bool     ok    = (range >= 100);   // heuristic: expect ≥100 µs swing

        Serial.print(F("   "));
        Serial.print(i);
        Serial.print(F("  | "));
        printPadded(lo,    5);
        Serial.print(F(" | "));
        printPadded(hi,    5);
        Serial.print(F(" | "));
        printPadded(range, 5);
        Serial.print(F(" | "));
        Serial.println(ok ? F("YES") : F("NO  <-- check wiring/surface"));
    }
    Serial.println();

    // ── Phase 3: live verification ────────────────────────────────────────
    Serial.println(F("PHASE 3 — Live Verification"));
    Serial.println(F("Move sensor over the line. Calibrated values: 0=white, 1000=black."));
    Serial.println(F("Press RESET on the board to re-run calibration."));
    Serial.println();
}

// ── loop ───────────────────────────────────────────────────────────────────
void loop()
{
    qtr.readCalibrated(sensorValues);

    // Per-sensor bar graph (calibrated 0–1000)
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        printBar(sensorValues[i], 1000, 8);
    }

    // Weighted-average position (0 = leftmost sensor, (N-1)*1000 = rightmost)
    uint32_t weightedSum = 0;
    uint32_t totalWeight = 0;
    bool     lineDetected = false;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        uint16_t v = sensorValues[i];
        if (v > 200) lineDetected = true;
        weightedSum += (uint32_t)v * i * 1000;
        totalWeight += v;
    }

    if (lineDetected && totalWeight > 0) {
        // position: 0–7000, centre = 3500
        int32_t pos = (int32_t)(weightedSum / totalWeight);
        // Convert to cm offset from sensor array centre
        float cm = ((float)pos / 1000.0f - (NUM_SENSORS - 1) / 2.0f) * SENSOR_SPACING_CM;
        Serial.print(F("  pos="));
        Serial.print(pos);
        Serial.print(F(" ("));
        if (cm >= 0.0f) Serial.print('+');
        Serial.print(cm, 1);
        Serial.print(F(" cm from centre)"));
    } else {
        Serial.print(F("  -- no line detected --"));
    }

    Serial.println();
    delay(100);
}
