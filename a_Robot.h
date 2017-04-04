#ifndef Robot_h
#define Robot_h

#include <Arduino.h>
#include "Usb.h"
#include "DualVNH5019MotorShield.h"
#include "Encoder.h"
#include <Servo.h>
#include "MDD10A.h"

//#include <Wire.h>
//#include "Adafruit_Sensor.h"
//#include "Adafruit_BNO055.h"
//#include "imumaths.h"

class Robot
{
public:
  // CONSTRUCTORS
  Robot(); // Default pin selection.


  void Read();
  void Write();
  void init();
  void SetLED(int red, int grn, int blu);
  //void SetController(Controller *controller);

  //Read Only Items
  
  float GetEncDriveRight();
  float GetEncDriveLeft();
  float GetPotLift();
  float GetGyroDegrees();
  float GetGyroAbsolute();

  //Readable/Writeable Items

  //Expect -400 through 400
  float DriveRightFrontSpeed;
  float DriveRightRearSpeed;
  float DriveLeftFrontSpeed;
  float DriveLeftRearSpeed;
  float LiftSpeed;
  float ClawSpeed;
  //float ArmSpeed;
  //float ClawPower;

  //Expect 0-255
  int LEDRed;
  int LEDBlue;
  int LEDGreen;

  int TorqueLimitDrive;
  int TorqueLimitLift;

  int startMillis = 0;

  bool setMillisTime = true;

 // int LiftPotVal = 0;

  USB Usb;

private:
  //Controller *controller;
  MDD10A mc;
  Servo serLiftLeft;
  Servo serLiftRight;
  Servo serClaw;

  
  Encoder encDriveRight;
  Encoder encDriveLeft;
 // Adafruit_BNO055 gyroLift; 

  float prevDriveRightFrontSpeed;
  float prevDriveRightRearSpeed;
  float prevDriveLeftFrontSpeed;
  float prevDriveLeftRearSpeed;
  float prevLiftSpeed;

  //Private Vars

  int _encDriveRight;
  int _encDriveLeft;
  int _potLift;
  float _gyroDegrees;

  //Pirvate Methods
  int convertToServo(float inVal);
  float torqueLimit(float prevVal, float curVal, int torqueLim);

  //ESC
  const int _servoMax = 150;
  const int _servoMin = 29;
  const int _servoNeut = 90;

  //Intake ESC Pin
  const int _liftLeft_PWM = 7;
  const int _liftRight_PWM = 5;
  const int _claw_PWM = 6;

  // Encoder Pinouts
  const int _drive_Right_encInt = 12;
  const int _drive_Right_encDig = 32;

  const int _drive_Left_encInt = 13;
  const int _drive_Left_encDig = 33;

  const int _potPin = 3;

  const int _gyroSDA = 20;
  const int _gyroSCL = 21;


};

#endif


