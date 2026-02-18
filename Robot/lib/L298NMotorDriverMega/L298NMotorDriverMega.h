// #pragma once

// #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//   #define TIMERS_AVAILABLE
// #endif

// #include <Arduino.h>

// class L298NMotorDriverMega
// {
//   public:
//     // CONSTRUCTORS
//     // User-defined pin selection.
//     L298NMotorDriverMega(unsigned char M1EN,
//                              unsigned char M1C,
//                              unsigned char M1D,
//                              unsigned char M2EN,
//                              unsigned char M2C,
//                              unsigned char M2D);

//     // PUBLIC METHODS
//     void init(); // Initialize pins and timer if applicable.
//     void setM1Speed(int speed); // Set speed for M1.
//     void setM2Speed(int speed); // Set speed for M2.
//     void setSpeeds(int m1Speed, int m2Speed); // Set speed for both Motors.
//     void flipM1(boolean flip); // Flip the direction of the speed for M1.
//     void flipM2(boolean flip); // Flip the direction of the speed for M2.
//     void setM1Brake(int brake); // brake motor 1
//     void setM2Brake(int brake);   // brake motor 2
//     void setBrakes(int m1Brake, int m2Brake); // set brakes on both motors

//   private:
//     unsigned char _M1PWM;
//     static const unsigned char _M1PWM_TIMER1_PIN = 11;
//     static const unsigned char _M1PWM_TIMER3_PIN = 5; 
//     static const unsigned char _M1PWM_TIMER4_PIN = 6; 
//     static const unsigned char _M1PWM_TIMER5_PIN = 45; 
//     unsigned char _M2PWM;
//     static const unsigned char _M2PWM_TIMER1_PIN = 12;  
//     static const unsigned char _M2PWM_TIMER3_PIN = 2;  
//     static const unsigned char _M2PWM_TIMER4_PIN = 7; 
//     static const unsigned char _M2PWM_TIMER5_PIN = 46;  
//     unsigned char _M1C;
//     unsigned char _M1D;
//     unsigned char _M2C;
//     unsigned char _M2D;
//     boolean _flipM1;
//     boolean _flipM2;
//     int Timer;
// };

#ifndef L298NMOTORDRIVERMEGA_H
#define L298NMOTORDRIVERMEGA_H

#include <Arduino.h>

class L298NMotorDriverMega
{
public:
  L298NMotorDriverMega(unsigned char M1EN,
                       unsigned char M1C,
                       unsigned char M1D,
                       unsigned char M2EN,
                       unsigned char M2C,
                       unsigned char M2D);

  void init();

  void setM1Speed(int speed);
  void setM2Speed(int speed);
  void setSpeeds(int m1Speed, int m2Speed);

  void flipM1(boolean flip);
  void flipM2(boolean flip);

  void setM1Brake(int brake);
  void setM2Brake(int brake);
  void setBrakes(int m1Brake, int m2Brake);

private:
  unsigned char _M1PWM, _M1C, _M1D;
  unsigned char _M2PWM, _M2C, _M2D;

  bool _flipM1;
  bool _flipM2;
};

#endif // L298NMOTORDRIVERMEGA_H
