#ifndef Lift_h
#define Lift_h

#include <Arduino.h>
#include "a_Robot.h"
//#include "PID_v1.h"

class Lift
{
public:
  // CONSTRUCTORS
  Lift(Robot *p); // Default pin selection.


  void Task();
  void init();


  int PotLowVal = 290;
  int PotHighVal = 880;
  int LiftDirection = 0; // 1 is up, 0 is stop, -1 is down

  bool UsePot = false;

  int SetPoint = 0;

  //Autonomous Methods
  //void LiftTo(double position);
  //void LiftAdd(double position);

  void SetTorqueLimit(int val);


  //PID

  //PID liftPID;

  //double liftCurPos = 0;
  //double liftSetPoint = 0;
  //double liftPIDOut = 0;

  float ControllerSpeed;



private:
  Robot *robot;


  //Lift PID
  /*
  bool liftPIDEnable = false;
  const int liftPIDTolerence = 0;
  const double liftKP = 60;
  const double liftKI = 0;
  const double liftKD = 1;
  */

};

#endif
