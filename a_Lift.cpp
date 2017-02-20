#include "a_Lift.h"
#include "a_Controller.h"
// Constructors ////////////////////////////////////////////////////////////////

Lift::Lift(Robot *p)
{
	robot = p;
	SetPoint = 0;
	UsePot = false;
	LiftDirection = 0;

}


void Lift::SetTorqueLimit(int val)
{
	robot->TorqueLimitLift = val;
}

void Lift::init()
{
	
}

void Lift::Task()
{



	if(ControllerSpeed != 0)
	{
		robot->LiftSpeed = ControllerSpeed;
		//liftSetPoint = liftCurPos;
	}

	if((robot->LiftPotVal >= PotHighVal && ControllerSpeed > 0 || robot->LiftPotVal <= PotLowVal && ControllerSpeed < 0) && UsePot){
	   robot->LiftSpeed = 0;
	   Serial.println(millis());
	}else 
	{
		if(SetPoint != 0){

			if(robot->LiftPotVal < SetPoint && LiftDirection >= 0){
				ControllerSpeed = 400;
				LiftDirection = 1;
			}else if(robot->LiftPotVal > SetPoint && LiftDirection <= 0){
				ControllerSpeed = -400;
				LiftDirection = -1;
			}
			else
			{
				ControllerSpeed = 0;
				LiftDirection = 0;
				SetPoint = 0;
			}

		}
	}

}
