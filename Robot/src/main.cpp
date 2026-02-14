#include <Arduino.h>
#include "Robot.hpp"

Robot robot;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(SERIAL_BAUD_RATE);
  robot.init();
}

void loop()
{
  robot.update();
}
