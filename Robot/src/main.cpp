#include <Arduino.h>
#include "Robot.hpp"
#include <SoftwareSerial.h>

Robot robot;

void setup()
{
  Serial.begin(9600);
  robot.init();
  Serial.println("Hello world");
}

void loop()
{
  robot.update();
}
