// Lab11Template.ino
// Prof. Stephen Mascaro
// 11/06/23

//  This sketch will be used to perform line localization and line following
#include <Encoder.h>
// #include <DualTB9051FTGMotorShieldUnoMega.h>
#include <L298NMotorDriverMega.h>

Encoder leftEncoder(18,19);  //Left encoder
Encoder rightEncoder(20,21);  //Right encoder

L298NMotorDriverMega motorDriver(
  6, 31, 33,  //Left Motor
  7, 35, 37); //Right Motor

double t, t_old, t0, deltaT, print_time=0; // declare some time variables

long countsLeft, countsRight;
int commandLeft=0, commandRight=0;  //declare and initialize motor commands
int commandAvg = 0;

double GearRatio = 50; 
int countsPerRev = 64; // encoder counts per Rev
double rw = 5; // wheel radius in cm
double D = 24; // distance between wheels in cm

double thetaLeft, omegaLeft, thetaLeft_old = 0;
double thetaRight, omegaRight, thetaRight_old = 0;
double omegaLeftf=0, omegaRightf=0, alpha=0.01; //digital filter
double mLeftcurrent, mRightcurrentf;
String inString = "";

void setup() {
  // initialize reflectance sensor and motor driver
  motorDriver.init(); 

  Serial.begin(9600);
   
  t0 = micros()/1000000.;
  t_old = 0;
}

void loop() {

  t = micros()/1000000.-t0;
  deltaT = t-t_old;  // sample time

  countsLeft = leftEncoder.read();
//  mLeftcurrent = md.getM1CurrentMilliamps()/1000.; //current in Amps
//  mRightcurrentf = alpha*m1current + (1-alpha)*m1currentf; //filtered current

  // calculate your position and velocity here
  thetaLeft = 2*PI*countsLeft/(countsPerRev*GearRatio);
  omegaLeft = (thetaLeft - thetaLeft_old)/deltaT;

  thetaRight = 2*PI*countsRight/(countsPerRev*GearRatio);
  omegaRight = (thetaRight - thetaRight_old)/deltaT;
  
  //Put step command here
  if ( t < 1 )
  {
    commandLeft = 200;
    commandRight = 200;
  }
  else{
    commandLeft = 0;
    commandRight = 0;
  }

  omegaAvg = (omegaLeft + omegaRight)/2.0;
  commandAvg = (commandLeft + commandRight)/2;

  motorDriver.setSpeeds(commandLeft,commandRight); // send motor commands
  // motorDriver.setSpeeds(commandAvg, commandAvg); // send motor commands
  
  // Put print commands here, make sure to use enough decimal places for your time and velocity
  Serial.print(t, 4); Serial.print('\t');
  Serial.print(omegaLeft, 4); Serial.print('\t');
  Serial.print(commandLeft, 4); Serial.print('\t');

  t_old = t;
  thetaLeft_old = thetaLeft;
}
