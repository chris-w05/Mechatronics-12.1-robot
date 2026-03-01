#include <Arduino.h>
#include "Robot.hpp"

Robot robot;

const int s0 = COLOR_SENSOR_PINS.s0_pin, 
          s1 = COLOR_SENSOR_PINS.s1_pin, 
          s2 = COLOR_SENSOR_PINS.s2_pin, 
          s3 = COLOR_SENSOR_PINS.s3_pin;

int readPin = COLOR_SENSOR_PINS.out_pin, LEDPin = COLOR_SENSOR_PINS.led_pin;

const int numSamples = 8;
float R[numSamples], G[numSamples], B[numSamples], C[numSamples]; // raw pulse time samples
float RF, GF, BF, CF;                                             // filtered data

float readPulse()
{
  return pulseIn(readPin, LOW) + pulseIn(readPin, HIGH);
}

float movingAverage(float *arr)
{
  float sum = 0;
  for (int i = 0; i < numSamples; i++)
  {
    sum += arr[i] / numSamples;
  }

  // Convert to frequency and return
  return 1000000 / sum;
}

// Red
//     RED : .93 B : .09 G : .05

//     Yellow
//         R : .57 B : .18 G : .3

//     Blue
//         R : .03 B : .75 G : .25

inline bool isRedBlock(float red, float blue, float green)
{
  return (red > .8f && red < 1.0f) &&
         (blue > 0.0f && blue < .2f) &&
         (green > .00f && green < .20f);
}
inline bool isYellowBlock(float red, float blue, float green)
{
  return (red > .4f && red < .70f) &&
         (blue > .10f && blue < .28f) &&
         (green > .2f && green < .40f);
}
inline bool isBlueBlock(float red, float blue, float green)
{
  return (red > .00f && red < .20f) &&
         (blue > .65f && blue < .9f) &&
         (green > .15f && green < .40f);
}

void getBlockColor(float red, float blue, float green)
{
  if (isBlueBlock(red, blue, green))
  {
    Serial.println("Blue block seen");
    return;
  }
  if (isRedBlock(red, blue, green))
  {
    Serial.println("Red block seen");
    return;
  }
  if (isYellowBlock(red, blue, green))
  {
    Serial.println("Yellow block seen");
    return;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(SERIAL_BAUD_RATE);
  robot.init();


  //Color sensor shit:

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(readPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(s0, HIGH); // s1 and s0 choose frequency scaling
  digitalWrite(s1, LOW);

  // digitalWrite(LEDPin, HIGH); // turn on LED
  Serial.println("Type in anything to the serial monitor to get a reading");
  Serial.println("Red\t\tGreen\t\tBlue\t\tClear");
}

void loop()
{

  robot.update();

  // Serial.print(">Hall effect signal:");
  // Serial.print( analogRead(HALL_EFFECT_PIN));
  // Serial.println("\r");
  if (!Serial.available())
  { // If no input given do nothing
    return;
  }
  else
  { // Otherwise flush serial buffer and proceed
    delay(5);
    while (Serial.available())
    {
      Serial.read();
    }
  }

  if (analogRead(HALL_EFFECT_PIN) > 670 ){
    Serial.println("Hall Effect: Magnet Detected");
  }
  else{
    Serial.println("Hall Effect: No Magnet detected");
  }

    //////// Take specified number of samples
    // Select RED Filter
    digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delay(10);
  for (int i = 0; i < numSamples; i++)
  {
    R[i] = readPulse();
  }

  // Select BLUE Filter
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delay(10);
  for (int i = 0; i < numSamples; i++)
  {
    B[i] = readPulse();
  }

  // Select GREEN Filter
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delay(10);
  for (int i = 0; i < numSamples; i++)
  {
    G[i] = readPulse();
  }

  // Select CLEAR Filter
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  delay(10);
  for (int i = 0; i < numSamples; i++)
  {
    C[i] = readPulse();
  }

  CF = movingAverage(C);
  RF = movingAverage(R)/CF;
  GF = movingAverage(G)/CF;
  BF = movingAverage(B)/CF;
  
  getBlockColor(RF, BF, GF);
  // Print Results

  // Serial.print(">Red:");
  // Serial.print(RF);
  // Serial.print(",Green:");
  // Serial.print(GF);
  // Serial.print(",Blue:");
  // Serial.print(BF);
  // Serial.println("\r");

  
  // Serial.print(RF, 2);
  // Serial.print(",\t");
  // Serial.print(GF, 2);
  // Serial.print(",\t");
  // Serial.print(BF, 2);
  // Serial.print(",\t");
  // Serial.println(CF, 2); // Check if ready to read

  
}

