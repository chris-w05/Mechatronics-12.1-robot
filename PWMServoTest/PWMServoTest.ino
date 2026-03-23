#include <Servo.h>

Servo myservo;
int pos = 0;

void setup() {
  myservo.attach(10);
  Serial.begin(9600);
  Serial.println("Enter an angle (0-180):");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();
    
    // Flush any leftover characters (newline, carriage return, etc.)
    while (Serial.available() > 0) Serial.read();
    
    if (angle >= 0 && angle <= 180) {
      myservo.write(angle);
      Serial.print("Moving to: ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid angle. Please enter a value between 0 and 180.");
    }
  }
}