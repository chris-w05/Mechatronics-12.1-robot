#include <PWMServo.h>

PWMServo myservo;
int pos = 0;

void setup() {
  myservo.attach(12);
  Serial.begin(9600);
  Serial.println("Enter an angle (0-180):");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();
    
    // Flush any leftover characters (newline, carriage return, etc.)
    while (Serial.available() > 0) Serial.read();
    
      myservo.write(angle);
      Serial.print("Moving to: ");
      Serial.println(angle);

  }
}