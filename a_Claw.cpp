#include "a_Claw.h"

// Constructors ////////////////////////////////////////////////////////////////

Claw::Claw(Robot *p)
{
	robot = p;
}

void Claw::Task()
{
	if(ControllerSpeed == 1)
		robot->ClawSpeed = 400;
	else if(ControllerSpeed == -1)
		robot->ClawSpeed = -400;
	else
		robot->ClawSpeed = 0;

}
