/*
  Rangefinder Calibrator
  Sensor -> analog pin A4

  Model: D = (V / a)^b
  Linearize: ln(D) = b * ln(V) + c  where c = -b * ln(a)
  Solve for b and c with linear regression on (ln(V), ln(D)),
  then a = exp(-c / b)

  How to use:
  - After upload open Serial Monitor at 115200 baud.
  - When prompted, enter distances in the units you want (e.g. cm) as comma-separated values
    or press Enter to use defaults.
  - For each distance: place the object at that distance and press ENTER (or follow prompts).
*/

const int ANALOG_PIN = A4;          // analog pin for the sensor
const float VREF = 5.0;            // analog reference voltage (change to 3.3 if using 3.3V board)
const int ADC_MAX = 1023;          // 10-bit ADC

const int DEFAULT_SAMPLE_COUNT = 200; // how many samples to average per distance
const int SAMPLE_DELAY_MS = 5;        // delay between individual analog reads

const float sensor_offset = 1.5;

const int MAX_POINTS = 30;          // maximum number of calibration distances supported

// storage for distances and measured voltages
float knownDistances[MAX_POINTS];
float measuredVoltages[MAX_POINTS];
int pointsCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait a little for serial */ }

  Serial.println();
  Serial.println(F("=== Rangefinder Calibration Utility ==="));
  Serial.println(F("Sensor on analog pin A4"));
  Serial.println(F("Model: D = (V / a)^b  (D = distance, V = measured voltage)"));
  Serial.println();
  Serial.println(F("Enter comma-separated known distances (units you choose, e.g. cm)."));
  Serial.println(F("Example: 10,20,30,40"));
  Serial.println(F("Or press Enter to use default distances."));
  Serial.println(F("You will be asked to place the object at each distance when prompted."));
  Serial.println();
  Serial.print(F("Enter distances now (10s to type)... "));

  // defaults (in the chosen unit, e.g. cm)
  float distancesToMeasure[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  pointsCount = sizeof(distancesToMeasure) / sizeof(distancesToMeasure[0]);
  for (int i = 0; i < pointsCount; ++i) knownDistances[i] = distancesToMeasure[i];
  Serial.println();
  Serial.println(F("No input detected — using default distances: 10,20,30,40,50"));

  // Sample at each distance
  for (int i = 0; i < pointsCount; ++i) {
    Serial.println();
    Serial.print(F("Place object at distance = "));
    Serial.print(knownDistances[i] - sensor_offset);
    Serial.println(F("  and press ENTER to record samples..."));
    waitForEnter(); // wait for user to press ENTER
    float avgV = sampleAverageVoltage(DEFAULT_SAMPLE_COUNT, SAMPLE_DELAY_MS);
    measuredVoltages[i] = avgV;
    Serial.print(F("Recorded average voltage: "));
    Serial.print(avgV, 5);
    Serial.println(F(" V"));
  }

  Serial.println();
  Serial.println(F("Fitting model ln(D) = b * ln(V) + c ..."));
  fitAndReport();
}

void loop() {
  // nothing to do; calibration runs in setup
  delay(1000);
}


// ---------- Utility Functions ----------

String readLineWithTimeout(unsigned long timeoutMs) {
  unsigned long start = millis();
  String s;
  while (millis() - start < timeoutMs) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') return s;
      s += c;
    }
    delay(5);
  }
  // times up; clear any leftover input
  while (Serial.available()) Serial.read();
  return s;
}

void parseDistances(const String& line) {
  pointsCount = 0;
  String token;
  for (unsigned int i = 0; i < line.length(); ++i) {
    char c = line.charAt(i);
    if (c == ',' || c == ' ' || c == '\t') {
      if (token.length() > 0) {
        if (pointsCount < MAX_POINTS) knownDistances[pointsCount++] = token.toFloat();
        token = "";
      }
    } else {
      token += c;
    }
  }
  if (token.length() > 0 && pointsCount < MAX_POINTS) {
    knownDistances[pointsCount++] = token.toFloat();
  }
}

void waitForEnter() {
  while (true) {
    if (Serial.available()) {
      int c = Serial.read();
      if (c == '\n') break;
    }
    delay(10);
  }
}

float sampleAverageVoltage(int samples, int delayMs) {
  long sum = 0;
  int validSamples = 0;
  for (int i = 0; i < samples; ++i) {
    int raw = analogRead(ANALOG_PIN);
    sum += raw;
    validSamples++;
    delay(delayMs);
  }
  float avgRaw = (float)sum / (float)validSamples;
  float volts = avgRaw * VREF / (float)ADC_MAX;
  return volts;
}


// ---------- Fitting ----------

void fitAndReport() {
  // Prepare ln(V) and ln(D) arrays
  double x[MAX_POINTS]; // ln(V)
  double y[MAX_POINTS]; // ln(D)
  int n = 0;
  for (int i = 0; i < pointsCount; ++i) {
    float V = measuredVoltages[i];
    float D = knownDistances[i];
    if (V <= 0.0f || D <= 0.0f) {
      Serial.print(F("Skipping point with non-positive V or D: "));
      Serial.print(F("D=")); Serial.print(D); Serial.print(F(" V=")); Serial.println(V);
      continue;
    }
    x[n] = log((double)V);
    y[n] = log((double)D);
    ++n;
  }

  if (n < 2) {
    Serial.println(F("Not enough valid points to fit."));
    return;
  }

  // Linear regression: y = b * x + c
  double sumx = 0, sumy = 0, sumxy = 0, sumx2 = 0;
  for (int i = 0; i < n; ++i) {
    sumx += x[i];
    sumy += y[i];
    sumxy += x[i] * y[i];
    sumx2 += x[i] * x[i];
  }
  double denom = (n * sumx2 - sumx * sumx);
  if (fabs(denom) < 1e-12) {
    Serial.println(F("Numerical problem computing regression (denominator zero)."));
    return;
  }
  double b = (n * sumxy - sumx * sumy) / denom;
  double c = (sumy - b * sumx) / n;

  // compute 'a' from c = -b * ln(a) => a = exp(-c / b)
  double a = exp(-c / b);

  // Compute R^2
  double ymean = sumy / n;
  double ss_tot = 0;
  double ss_res = 0;
  for (int i = 0; i < n; ++i) {
    double yi_hat = b * x[i] + c;
    ss_res += (y[i] - yi_hat) * (y[i] - yi_hat);
    ss_tot += (y[i] - ymean) * (y[i] - ymean);
  }
  double r2 = 1.0 - (ss_res / ss_tot);

  // Report
  Serial.println();
  Serial.println(F("=== Fit Results ==="));
  Serial.print(F("b (exponent) = ")); Serial.println(b, 8);
  Serial.print(F("a (scale)    = ")); Serial.println(a, 8);
  Serial.print(F("Model (human) : D = (V / a)^b  -> D = (V / "));
  Serial.print(a, 6);
  Serial.print(F(")^")); Serial.println(b, 6);
  Serial.print(F("Model (program): float distance(float V) { return pow(V/"));
  Serial.print(a, 6);
  Serial.print(F(", ")); Serial.print(b, 6);
  Serial.println(F("); }"));
  Serial.print(F("R^2 (fit quality) = ")); Serial.println(r2, 6);
  Serial.println();

  Serial.println(F("Data and residuals (Distance, Volts, PredictedDistance, Error):"));
  for (int i = 0, k = 0; i < pointsCount; ++i) {
    float V = measuredVoltages[i];
    float D = knownDistances[i];
    if (V <= 0 || D <= 0) continue;
    double pred_lnD = b * log((double)V) + c;
    double predD = exp(pred_lnD);
    double err = predD - (double)D;
    Serial.print(F("D=")); Serial.print(D, 6);
    Serial.print(F("  V=")); Serial.print(V, 6);
    Serial.print(F("  predD=")); Serial.print(predD, 6);
    Serial.print(F("  err=")); Serial.println(err, 6);
    k++;
  }

  Serial.println();
  Serial.println(F("Example: To compute distance from raw analog reading:"));
  Serial.println(F("  int raw = analogRead(A4);"));
  Serial.println(F("  float V = raw * Vref / 1023.0;"));
  Serial.print(F("  float D = pow(V / ")); Serial.print(a,6); Serial.print(F(", ")); Serial.print(b,6); Serial.println(F(");"));

  Serial.println();
  Serial.println(F("Calibration finished."));
}