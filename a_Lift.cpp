#include "a_Lift.h"
#include "a_Controller.h"
// Constructors ////////////////////////////////////////////////////////////////

Lift::Lift(Robot *p)
{
	robot = p;
}


void Lift::SetLiftKP(double val)
{
	liftPID.SetTunings(val, liftPID.GetKi(), liftPID.GetKd());
}
void Lift::SetLiftKI(double val)
{
	liftPID.SetTunings(liftPID.GetKp(), val, liftPID.GetKd());
}
void Lift::SetLiftKD(double val)
{
	liftPID.SetTunings(liftPID.GetKp(), liftPID.GetKi(), val);
}

void Lift::SetTorqueLimit(int val)
{
	robot->TorqueLimitLift = val;
}



void Lift::LiftTo(double position)
{
	liftSetPoint = position;
	liftPID.SetMode(AUTOMATIC);
}

void Lift::LiftAdd(double position)
{
	liftSetPoint += position;
	liftPID.SetMode(AUTOMATIC);
}



void Lift::init()
{
	liftPID.init(&liftCurPos, &liftPIDOut, &liftSetPoint, liftKP, liftKI, liftKD, DIRECT);
	liftCurPos = robot->GetPotLift();
	liftPID.SetMode(MANUAL);
	liftPIDOut = 0;
	liftPID.SetOutputLimits(-400, 400);

	useLimits = true;
}

void Lift::UseLimits(bool _useLimits)
{
	useLimits = _useLimits;
}

void Lift::Task()
{
	/*if (liftSetPoint == 0 && liftPID.GetMode() == AUTOMATIC)
	liftPID.SetMode(MANUAL);
	else if(liftPID.GetMode() == MANUAL && liftSetPoint != 0)
	liftPID.SetMode(AUTOMATIC);*/

	liftCurPos = robot->GetPotLift();
	liftPID.Compute();

	//Serial.print(liftSetPoint);



	if (ControllerSpeed != 0)
	{
		robot->LiftSpeed = ControllerSpeed;
		liftPID.SetMode(MANUAL);
		liftPIDOut = 0;
	}
	else
	{
		if (abs(liftSetPoint - liftCurPos) > liftPIDTolerence)
		{
			robot->LiftSpeed = liftPIDOut;
		}
		else
		{
			robot->LiftSpeed = 0;
		}
	}

	if ((robot->GetPotLift() >= PotHighVal && ControllerSpeed > 0 || robot->GetPotLift() <= PotLowVal && ControllerSpeed < 0) && useLimits)
		robot->LiftSpeed = 0;

	useLimits = true;

	/*
	if((robot->GetPotLift() >= PotHighVal && ControllerSpeed > 0 || robot->GetPotLift() <= PotLowVal && ControllerSpeed < 0) && UsePot){
	robot->LiftSpeed = 0;
	Serial.println(millis());
	}else
	{
	if(SetPoint != 0){

	if(robot->GetPotLift() < SetPoint && LiftDirection >= 0){
	ControllerSpeed = 400;
	LiftDirection = 1;
	}else if(robot->GetPotLift() > SetPoint && LiftDirection <= 0){
	ControllerSpeed = -400;
	LiftDirection = -1;
	}
	else
	{
	ControllerSpeed = 0;
	LiftDirection = 0;
	SetPoint = 0;
	}

	}else{ ControllerSpeed = 0;


	}
	}
	*/
}


