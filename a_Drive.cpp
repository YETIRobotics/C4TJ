#include "a_Drive.h"

// Constructors ////////////////////////////////////////////////////////////////

Drive::Drive(Robot *p)
{
	robot = p;
}

void Drive::SetTorqueLimit(int val)
{
	robot->TorqueLimitDrive = val;
}

void Drive::init()
{

}


void Drive::Task()
{

	if(LeftControllerSpeedY != 0 || RightControllerSpeedY != 0 || LeftControllerSpeedX != 0 || RightControllerSpeedX != 0)
	{

		//Joystick interpretation

		float Ch3 = LeftControllerSpeedY;
		float Ch1 = RightControllerSpeedX;
		float Ch4 = LeftControllerSpeedX;

		robot->DriveRightFrontSpeed = Ch3 - Ch1 - Ch4;
		robot->DriveRightRearSpeed = Ch3 - Ch1 + Ch4;
		robot->DriveLeftFrontSpeed = Ch3 + Ch1 + Ch4;
		robot->DriveLeftRearSpeed = Ch3 + Ch1 - Ch4;





		//driveLeftSetPoint = driveLeftCurPos - correctionVal;
		//driveRightSetPoint = driveRightCurPos - correctionVal;
	}else{
		robot->DriveRightFrontSpeed = 0;
		robot->DriveRightRearSpeed = 0;
		robot->DriveLeftFrontSpeed = 0;
		robot->DriveLeftRearSpeed = 0;
	}

}


