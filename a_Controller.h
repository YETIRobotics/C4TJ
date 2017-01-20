#ifndef Controller_h
#define Controller_h

#include <Arduino.h>
#include "XBOXRECV.h"


class Controller
{
  public:
    // CONSTRUCTORS
    Controller();
    Controller(USB *pUsb); // Default pin selection.

    void Task();


    float LeftJoystickY;
    float LeftJoystickX;
    float RightJoystickY;
    float RightJoystickX;
    float TriggerAggregate;
    int LR2Aggregate;

    int DPadLeftRight;
    int APress;
    int YPress;

   private:

    XBOXRECV Xbox;
    const int joystickMinThresh = 7000;
    USB dummyXboxPointer;

};

#endif
