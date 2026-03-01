#include <Arduino.h>
#include "Robot.hpp"

Robot robot;

// unsigned long lastTime = 0.0;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(SERIAL_BAUD_RATE);
  robot.init();
}

void loop()
{
  //Cycle time debug
  // unsigned long now = micros();
  // Serial.print(">CycleTime:");
  // Serial.print(now - lastTime);
  // Serial.println("\r");
  // lastTime = now;
  robot.update();
}
