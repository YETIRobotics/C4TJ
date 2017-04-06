#include "a_Drive.h"

// Constructors ////////////////////////////////////////////////////////////////

Drive::Drive(Robot *p)
{
	robot = p;
}

void Drive::SetHLKP(double val)
{
	hlPID.SetTunings(val, hlPID.GetKi(), hlPID.GetKd());
}
void Drive::SetHLKI(double val)
{
	hlPID.SetTunings(hlPID.GetKp(), val, hlPID.GetKd());
}
void Drive::SetHLKD(double val)
{
	hlPID.SetTunings(hlPID.GetKp(), hlPID.GetKi(), val);
}

void Drive::SetDriveKP(double val)
{
	drivePID.SetTunings(val, drivePID.GetKi(), drivePID.GetKd());
}
void Drive::SetDriveKI(double val)
{
	drivePID.SetTunings(drivePID.GetKp(), val, drivePID.GetKd());
}
void Drive::SetDriveKD(double val)
{
	drivePID.SetTunings(drivePID.GetKp(), drivePID.GetKi(), val);
}

void Drive::SetTurnKP(double val)
{
	turnPID.SetTunings(val, turnPID.GetKi(), turnPID.GetKd());
}
void Drive::SetTurnKI(double val)
{
	turnPID.SetTunings(turnPID.GetKp(), val, turnPID.GetKd());
}
void Drive::SetTurnKD(double val)
{
	turnPID.SetTunings(turnPID.GetKp(), turnPID.GetKi(), val);
}

void Drive::SetTorqueLimit(int val)
{
	robot->TorqueLimitDrive = val;
}

void Drive::Move(double position)
{
	turnPID.SetMode(MANUAL);
	turnPIDOut = 0;

	driveSetPoint = robot->GetEncDriveRight() + position;
	drivePID.SetMode(AUTOMATIC);
}

void Drive::Turn(double degrees)
{
	hlPID.SetMode(MANUAL);
	hlPIDOut = 0;
	drivePID.SetMode(MANUAL);
	drivePIDOut = 0;

	turnSetPoint = robot->GetGyroDegrees() + degrees;
	turnPID.SetMode(AUTOMATIC);
}

void Drive::TurnTo(double degrees)
{
	hlPID.SetMode(MANUAL);
	hlPIDOut = 0;
	drivePID.SetMode(MANUAL);
	drivePIDOut = 0;

	turnSetPoint = degrees;
	turnPID.SetMode(AUTOMATIC);
}

void Drive::HeadingLockEnable()
{
	_headingLockEnabled = true;
	hlPID.SetMode(AUTOMATIC);
	hlSetPoint = robot->GetGyroDegrees();
}

void Drive::HeadingLockDisable()
{
	_headingLockEnabled = false;
	hlPID.SetMode(MANUAL);
	hlPIDOut = 0;
}

void Drive::HeadingLockToggle()
{
	if (_headingLockEnabled)
		HeadingLockDisable();
	else
		HeadingLockEnable();
}

void Drive::init()
{
	hlPID.init(&hlCurPos, &hlPIDOut, &hlSetPoint, hlKP, hlKI, hlKD, REVERSE);
	hlCurPos = robot->GetGyroDegrees();
	hlPID.SetMode(MANUAL);
	hlPIDOut = 0;
	hlPID.SetOutputLimits(-60, 60);

	drivePID.init(&driveCurPos, &drivePIDOut, &driveSetPoint, driveKP, driveKI, driveKD, DIRECT);
	driveCurPos = robot->GetEncDriveRight();
	drivePID.SetMode(MANUAL);
	drivePIDOut = 0;
	drivePID.SetOutputLimits(-400, 400);

	turnPID.init(&turnCurPos, &turnPIDOut, &turnSetPoint, turnKP, turnKI, turnKD, REVERSE);
	turnCurPos = robot->GetGyroDegrees();
	turnPID.SetMode(MANUAL);
	turnPIDOut = 0;
	turnPID.SetOutputLimits(-400, 400);
}


void Drive::Task()
{

	hlCurPos = robot->GetGyroDegrees();
	hlPID.Compute();

	driveCurPos = robot->GetEncDriveRight();
	drivePID.Compute();

	turnCurPos = robot->GetGyroDegrees();
	turnPID.Compute();


	if (LeftControllerSpeedY != 0 || RightControllerSpeedY != 0 || LeftControllerSpeedX != 0 || RightControllerSpeedX != 0)
	{

		drivePID.SetMode(MANUAL);
		drivePIDOut = 0;

		turnPID.SetMode(MANUAL);
		turnPIDOut = 0;

		if (RightControllerSpeedX != 0)
		{
			hlSetPoint = robot->GetGyroDegrees();
		}

		//Joystick interpretation

		float Ch3 = LeftControllerSpeedY;
		float Ch1 = RightControllerSpeedX;
		float Ch4 = LeftControllerSpeedX;

		float rf = Ch3 - Ch1 - Ch4;
		float rr = Ch3 - Ch1 + Ch4;
		float lf = Ch3 + Ch1 + Ch4;
		float lr = Ch3 + Ch1 - Ch4;

		robot->DriveRightFrontSpeed = rf + hlPIDOut;
		robot->DriveRightRearSpeed = rr + hlPIDOut;
		robot->DriveLeftFrontSpeed = lf - hlPIDOut;
		robot->DriveLeftRearSpeed = lr - hlPIDOut;

		//Time for heading correction
		//You will get -100 - 100 from PID for correction.



		//driveLeftSetPoint = driveLeftCurPos - correctionVal;
		//driveRightSetPoint = driveRightCurPos - correctionVal;
	}
	else {
		if (abs(driveSetPoint - driveCurPos) > drivePIDTolerence && drivePID.GetMode() == AUTOMATIC)
		{

			robot->DriveRightFrontSpeed = drivePIDOut + hlPIDOut;
			robot->DriveRightRearSpeed = drivePIDOut + hlPIDOut;
			robot->DriveLeftFrontSpeed = drivePIDOut - hlPIDOut;
			robot->DriveLeftRearSpeed = drivePIDOut - hlPIDOut;
		}
		else if (abs(turnSetPoint - turnCurPos) > turnPIDTolerence && turnPID.GetMode() == AUTOMATIC)
		{
			robot->DriveRightFrontSpeed = turnPIDOut;
			robot->DriveRightRearSpeed = turnPIDOut;
			robot->DriveLeftFrontSpeed = -turnPIDOut;
			robot->DriveLeftRearSpeed = -turnPIDOut;

			hlSetPoint = robot->GetGyroDegrees();
		}
		else
		{
			robot->DriveRightFrontSpeed = 0;
			robot->DriveRightRearSpeed = 0;
			robot->DriveLeftFrontSpeed = 0;
			robot->DriveLeftRearSpeed = 0;
		}

	}

}


