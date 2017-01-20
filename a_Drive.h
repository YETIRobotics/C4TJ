#ifndef Drive_h
#define Drive_h

#include <Arduino.h>
#include "a_Robot.h"
//#include "PID_v1.h"

class Drive
{
  public:
    // CONSTRUCTORS
    Drive(Robot *p); // Default pin selection.

    void Task();
    void init();




    //Autonomous Methods
    /*
    void DriveLeft(double position);
    void DriveRight(double position);
    void Move(double position);
    */

    void SetTorqueLimit(int val);


    //Floats
    float LeftControllerSpeedY;
    float LeftControllerSpeedX;
    float RightControllerSpeedY;
    float RightControllerSpeedX;

    //DriveLeftPID

  private:
  	Robot *robot;

    int correctionVal = 0;

};

#endif
