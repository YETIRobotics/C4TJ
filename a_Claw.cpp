#include "a_Claw.h"

// Constructors ////////////////////////////////////////////////////////////////

Claw::Claw(Robot *p)
{
	robot = p;
	ClampTime = 0;
}

void Claw::Task()
{
	if(ControllerSpeed == 1)
		robot->ClawSpeed = 400;
	else if(ControllerSpeed == -1)
		robot->ClawSpeed = -400;
	else
		robot->ClawSpeed = 0;

	if(ControllerSpeed != 0 && ClampTime == 0)
	{
		StartClampTime = millis();
	}

	ClampTime = millis() - StartClampTime;

	if(ControllerSpeed != 0 && ClampTime >= 1500){
		robot->ClawSpeed *= 0.007;
	}

	if(ControllerSpeed == 0){
		ClampTime = 0;
		StartClampTime = millis();

	}

}
