#ifndef CLaw_h
#define Claw_h

#include <Arduino.h>
#include "a_Robot.h"

class Claw
{
public:
	// CONSTRUCTORS
	Claw(Robot *p); // Default pin selection.

	bool autoMove = false;
	float autoMillis = 0;
	float speed = 0;
	float autoStart = 0;
	float autoSpeed = 0;

	void Task();
	void Move(float speed, float mSec);
	void Open();
	void Deploy();
	void Close();
	void Clamp();


	int ControllerSpeed;

	int StartClampTime = 0;
	int ClampTime = 0;

private:
	Robot *robot;

	bool _shouldClamp = false;

};

#endif