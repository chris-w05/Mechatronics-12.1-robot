// Simple quadrature test for Arduino Mega
volatile long encCount = 0;
volatile uint8_t lastAB = 0;

const uint8_t A_PIN = 2; // change to 2 or 3 for a quick test
const uint8_t B_PIN = 3;

void IRAM_ATTR handleA() {
  uint8_t a = digitalRead(A_PIN);
  uint8_t b = digitalRead(B_PIN);
  uint8_t cur = (a<<1) | b;
  // quadrature decode: compare previous to current
  // Gray-code approach: steps +1 or -1
  if ((lastAB == 0b00 && cur == 0b01) ||
      (lastAB == 0b01 && cur == 0b11) ||
      (lastAB == 0b11 && cur == 0b10) ||
      (lastAB == 0b10 && cur == 0b00)) {
    encCount++;
  } else if ((lastAB == 0b00 && cur == 0b10) ||
             (lastAB == 0b10 && cur == 0b11) ||
             (lastAB == 0b11 && cur == 0b01) ||
             (lastAB == 0b01 && cur == 0b00)) {
    encCount--;
  }
  lastAB = cur;
}

void IRAM_ATTR handleB() { // same as A handler
  handleA();
}

void setup() {
  Serial.begin(115200);
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  lastAB = (digitalRead(A_PIN)<<1) | digitalRead(B_PIN);
  attachInterrupt(digitalPinToInterrupt(A_PIN), handleA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_PIN), handleB, CHANGE);
  Serial.println("Quadrature test started");
}

unsigned long lastT = 0;
long lastCount = 0;
void loop() {
  unsigned long t = millis();
  if (t - lastT >= 100) {
    noInterrupts();
    long c = encCount;
    interrupts();
    long delta = c - lastCount;
    float ticks_per_sec = (delta) * (1000.0 / (t - lastT));
    Serial.print("count=");
    Serial.print(c);
    Serial.print("   ticks/s=");
    Serial.println(ticks_per_sec);
    lastCount = c;
    lastT = t;
  }
}
