#include <Arduino.h>
#include "Robot.hpp"

Robot robot;

// unsigned long lastTime = 0.0;

/**
 * Arduino Setup
 * 
 * EVERYTHING OTHER THAN SERIAL SHOULD BE INIT FRO CHILDREN OF ROBOT.INIT()
 */
void setup()
{
  Serial.begin(115200);
  Serial2.begin(SERIAL_BAUD_RATE);
  robot.init();
  pinMode(48, OUTPUT);
  digitalWrite(48, HIGH);
}


/**
 * Arduino main loop. 
 * 
 * DO NOT PUT ANYTHING HERE OUTSIDE OF THE ROBOT
 */
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
