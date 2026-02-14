// UNO_XBEE_FIXED.ino
#include <Arduino.h>
#include <SoftwareSerial.h>

// Use pins 2 (RX) and 3 (TX) for SoftwareSerial
const uint8_t RX_PIN = 2; // connect XBee TX -> Arduino pin 2
const uint8_t TX_PIN = 3; // connect XBee RX <- Arduino pin 3
SoftwareSerial xbee(RX_PIN, TX_PIN);

void setup()
{
  Serial.begin(115200); // USB serial to PC
  delay(50);
  xbee.begin(115200); // XBee serial (match the XBee baud in XCTU)
  Serial.println("UNO ready. Type lines in Serial Monitor to send to XBee.");
}

void loop()
{
  // Send lines typed into Serial Monitor to XBee
  if (Serial.available())
  {
    String s = Serial.readStringUntil('\n'); // read a line from USB
    xbee.println(s);                         // send to XBee
    Serial.print("TX->XBee: ");
    Serial.println(s);
  }

  // Forward anything coming from XBee to Serial Monitor
  while (xbee.available())
  {
    int c = xbee.read();
    Serial.write((char)c);
  }
}
