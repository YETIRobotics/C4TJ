#include "a_Claw.h"

// Constructors ////////////////////////////////////////////////////////////////

Claw::Claw(Robot *p)
{
	robot = p;
	ClampTime = 0;
}

void Claw::Clamp()
{
	_shouldClamp = true;
	Move(400, 250);
}

void Claw::Close()
{
	_shouldClamp = false;
	Move(400, 250);
}

void Claw::Open()
{
	_shouldClamp = false;
	Move(-400, 400);
}

void Claw::Deploy()
{
	_shouldClamp = false;
	Move(-400, 200);
}

void Claw::Move(float speed, float mSec) {
	autoSpeed = speed;
	autoMillis = mSec;
	autoStart = millis();
	autoMove = true;
}

void Claw::Task()
{
	if (_shouldClamp)
	{
		robot->ClawSpeed = 200;
	}
	else
	{
		robot->ClawSpeed = 0;
	}

	if (ControllerSpeed == 1) {
		robot->ClawSpeed = 400;
		autoMove = false;
		autoMillis = 0;
		autoSpeed = 0;
		_shouldClamp = true;
	}
	else if (ControllerSpeed == -1) {
		robot->ClawSpeed = -400;
		autoMove = false;
		autoMillis = 0;
		autoSpeed = 0;
		_shouldClamp = false;
	}
	else if (millis() - autoStart >= autoMillis) {
		autoMove = false;
		autoMillis = 0;
		autoSpeed = 0;
	}

	if (autoMove) {
		robot->ClawSpeed = autoSpeed;
	}



	/*
	if(ControllerSpeed != 0 && ClampTime == 0)
	{
	StartClampTime = millis();
	}

	ClampTime = millis() - StartClampTime;

	if(ControllerSpeed != 0 && ClampTime >= 1500){
	robot->ClawSpeed *= 0.1;
	}

	if(ControllerSpeed == 0){
	ClampTime = 0;
	StartClampTime = millis();

	}
	*/

	//ControllerSpeed *= 0.1;

}


