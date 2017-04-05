#ifndef Drive_h
#define Drive_h

#include <Arduino.h>
#include "a_Robot.h"
#include "PID_v1.h"

class Drive
{
public:
	// CONSTRUCTORS
	Drive(Robot *p); // Default pin selection.

	void Task();
	void init();

	void HeadingLockEnable();
	void HeadingLockDisable();
	void HeadingLockToggle();

	//Autonomous Methods
	void Turn(double degrees);
	void Move(double position);

	void SetTorqueLimit(int val);

	void SetHLKP(double val);
	void SetHLKI(double val);
	void SetHLKD(double val);
	void SetDriveKP(double val);
	void SetDriveKI(double val);
	void SetDriveKD(double val);
	void SetTurnKP(double val);
	void SetTurnKI(double val);
	void SetTurnKD(double val);

	//Floats
	float LeftControllerSpeedY;
	float LeftControllerSpeedX;
	float RightControllerSpeedY;
	float RightControllerSpeedX;

	PID hlPID;
	double hlCurPos = 0;
	double hlSetPoint = 0;
	double hlPIDOut = 0;
	double hlCorrSetPoint = 0;

	PID drivePID;
	double driveCurPos = 0;
	double driveSetPoint = 0;
	double drivePIDOut = 0;
	double driveCorrSetPoint = 0;

	PID turnPID;
	double turnCurPos = 0;
	double turnSetPoint = 0;
	double turnPIDOut = 0;
	double turnCorrSetPoint = 0;


private:
	Robot *robot;

	int correctionVal = 0;

	bool _headingLockEnabled = false;
	const int hlPIDTolerence = 0;
	const double hlKP = 15;
	const double hlKI = 0;
	const double hlKD = .2;

	bool _drivePIDEnabled = false;
	const int drivePIDTolerence = 10;
	const double driveKP = 2;
	const double driveKI = 0;
	const double driveKD = .2;

	bool _turnPIDEnabled = false;
	const int turnPIDTolerence = 10;
	const double turnKP = 8;
	const double turnKI = 0;
	const double turnKD = .4;
};

#endif


