#ifndef CLaw_h
#define Claw_h

#include <Arduino.h>
#include "a_Robot.h"

class Claw
{
  public:
    // CONSTRUCTORS
    Claw(Robot *p); // Default pin selection.

    void Task();


    int ControllerSpeed;

    int StartClampTime = 0;
    int ClampTime = 0;

  private:
  	Robot *robot;

};

#endif


