/**
 * @file main.cpp
 * @brief Arduino entry point — instantiates the Robot and drives the main loop.
 *
 * All hardware initialisation is delegated to `Robot::init()` and all
 * per-loop work is delegated to `Robot::update()`.  This file should remain
 * as minimal as possible.
 */
#include <Arduino.h>
#include "Robot.hpp"

Robot robot;

unsigned long loopCount;
unsigned long lastTime = 0;

// unsigned long lastTime = 0.0;

/**
 * @brief Arduino `setup()` — called once on power-on or reset.
 *
 * Opens the USB serial port at 115200 baud, opens Serial2 for
 * Arduino-to-Arduino comms, and delegates all remaining initialisation
 * to `Robot::init()`.
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
 * @brief Arduino `loop()` — called repeatedly after `setup()`.
 *
 * All work is performed inside `Robot::update()`.
 * Do not add logic here; extend `Robot` or its subsystems instead.
 */
void loop()
{
  //Cycle time debug
  // unsigned long now = micros();
  // Serial.print(">CycleTime:");
  // Serial.print(now - lastTime);
  // Serial.println("\r");
  // lastTime = now;
  // loopCount ++;

  // if (loopCount % 1000 == 0){
  //   Serial.print("Average loop time (ms): ");
  //   Serial.print( (millis() - lastTime) / (float)loopCount);
  //   Serial.println();
  //   lastTime = millis();
  //   loopCount = 0;
  // }
  robot.update();
}
