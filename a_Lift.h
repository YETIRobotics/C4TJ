#ifndef Lift_h
#define Lift_h

#include <Arduino.h>
#include "a_Robot.h"
#include "PID_v1.h"

class Lift
{
public:
	// CONSTRUCTORS
	Lift(Robot *p); // Default pin selection.


	void Task();
	void init();


	int PotLowVal = 290;
	int PotHighVal = 880;
	int PotHighFence = 790;
	int PotLowFence = 715;

	void UseLimits(bool useLimits);

	//Autonomous Methods
	void LiftTo(double position);
	void LiftAdd(double position);

	void SetTorqueLimit(int val);

	void SetLiftKP(double val);
	void SetLiftKI(double val);
	void SetLiftKD(double val);


	//PID

	PID liftPID;

	double liftCurPos = 0;
	double liftSetPoint = 0;
	double liftPIDOut = 0;

	float ControllerSpeed;



private:
	Robot *robot;

	bool useLimits;

	//Lift PID

	bool liftPIDEnable = false;
	const int liftPIDTolerence = 0;
	const double liftKP = 3;
	const double liftKI = .1;
	const double liftKD = .1;


};

#endif


