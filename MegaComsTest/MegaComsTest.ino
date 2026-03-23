// MEGA_XBEE.ino
#include <Arduino.h>

void setup() {
  Serial.begin(9600);   // USB monitor
  Serial1.begin(115200);  // Serial1 to XBee (RX1/TX1 hardware pins)
  Serial.println("MEGA ready. Data from XBee will show below.");
}

void loop() {
  // XBee -> USB
  if (Serial1.available()) {
    char c = (char)Serial1.read();
    Serial.write(c);

  }

  // USB -> XBee
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    Serial1.println(s); // send to XBee
    Serial.print("TX->XBee: ");
    Serial.println(s);
  }
}