

#include "a_Robot.h"

// Constructors ////////////////////////////////////////////////////////////////

Robot::Robot()

{
	Usb.Init();

	mc1.init(_mc1_INA1, _mc1_INB1, _mc1_EN1DIAG1, _mc1_CS1, _mc1_INA2, _mc1_INB2, _mc1_EN2DIAG2, _mc1_CS2, _mc1_PWM1, _mc1_PWM2);
	mc2.init(_mc2_INA1, _mc2_INB1, _mc2_EN1DIAG1, _mc2_CS1, _mc2_INA2, _mc2_INB2, _mc2_EN2DIAG2, _mc2_CS2, _mc2_PWM1, _mc2_PWM2);

	/*
	encDriveRight.init(_drive_Right_encInt, _drive_Right_encDig);
	encDriveLeft.init(_drive_Left_encInt, _drive_Left_encDig);
	encLiftRight.init(_lift_Right_encInt, _lift_Right_encDig);
	encLiftLeft.init(_lift_Left_encInt, _lift_Left_encDig);
	encClaw.init(_claw_encInt, _claw_encDig);
	*/

	prevDriveRightFrontSpeed = 0;
	prevDriveRightRearSpeed = 0;
	prevDriveLeftFrontSpeed = 0;
	prevDriveLeftRearSpeed = 0;

	DriveRightFrontSpeed = 0;
	DriveRightRearSpeed = 0;
	DriveLeftFrontSpeed = 0;
	DriveLeftRearSpeed = 0;
	LiftLeftSpeed = 0;
	LiftRightSpeed = 0;
	ClawSpeed = 0;

	//Expect 0-255
	LEDRed = 0;
	LEDBlue = 0;
	LEDGreen = 0;

	TorqueLimitDrive = 0;
	TorqueLimitLift = 0;

}

void Robot::init(){

	serIntake.attach(_intake_PWM);
	serIntake.write(90);

	serArm.attach(_arm_PWM);
	serArm.write(90);

	serClaw.attach(_claw_PWM);
	serClaw.write(90);

	pinMode(_ledGrn, OUTPUT);
	pinMode(_ledBlu, OUTPUT);
	pinMode(_ledRed, OUTPUT);

	analogWrite(_ledRed, LEDRed);
	analogWrite(_ledGrn, LEDGreen);
	analogWrite(_ledBlu, LEDBlue);
}

void Robot::Read(){

	Usb.Task();

	/*
	_encDriveRight = encDriveRight.read() * -1;
	_encDriveLeft = encDriveLeft.read();
	_encLiftRight = encLiftRight.read();
	_encLiftLeft = encLiftLeft.read() * -1;
	_encClaw = encClaw.read();
	_leftLimitSwitch = digitalRead(_lift_Left_Limit);
	_rightLimitSwitch = digitalRead(_lift_Right_Limit);
	*/

	_driveLeftRearCurrent = mc2.getM2CurrentMilliamps();
	_driveLeftFrontCurrent = mc2.getM1CurrentMilliamps();
	_driveRightRearCurrent = mc1.getM2CurrentMilliamps();
	_driveRightFrontCurrent = mc1.getM1CurrentMilliamps();






}
void Robot::SetLED(int red, int grn, int blu)
{
	LEDRed = red;
	LEDBlue = blu;
	LEDGreen = grn;
}

float Robot::torqueLimit(float prevVal, float curVal, int torqueLim)
{
	float retVal = 0;

	//if torqueLim > 0, then it's enabled
	if(torqueLim > 0)
	{
		//If change is from negative to positive, or positive to negative
		//then pretend we were previously at zero.
		if((curVal > 0 && prevVal < 0) || (curVal < 0 && prevVal > 0))
		prevVal = 0;


		//if is increase in power that is greater than torque limit
		if(prevVal >= 0 && curVal > 0 && curVal > prevVal && curVal > (prevVal+torqueLim))
		{
			//increase forward
			retVal = prevVal + torqueLim;
		}
		else if(prevVal <= 0 && curVal < 0 && curVal < prevVal && curVal < (prevVal-torqueLim))
		{
			//increase in reverse
			retVal = prevVal - torqueLim;
		}
		else //decrease in power
		{
			retVal = curVal;
		}
	}
	else
	{
		retVal = curVal;
	}

	return retVal;
}

void Robot::Write(){

	//DriveRightSpeed
	if(DriveRightFrontSpeed < -400)
	DriveRightFrontSpeed = -400;
	if(DriveRightFrontSpeed > 400)
	DriveRightFrontSpeed = 400;

	if(DriveRightRearSpeed < -400)
	DriveRightRearSpeed = -400;
	if(DriveRightRearSpeed > 400)
	DriveRightRearSpeed = 400;

	prevDriveRightFrontSpeed = torqueLimit(prevDriveRightFrontSpeed, DriveRightFrontSpeed, TorqueLimitDrive);

	mc1.setM1Speed(prevDriveRightFrontSpeed);

	prevDriveRightRearSpeed = torqueLimit(prevDriveRightRearSpeed, DriveRightRearSpeed, TorqueLimitDrive);

	mc1.setM2Speed(prevDriveRightRearSpeed);

	//Serial.print(DriveRightSpeed);
	//Serial.print("\t");

	//DriveLeftSpeed
	if(DriveLeftFrontSpeed < -400)
	DriveLeftFrontSpeed = -400;
	if(DriveLeftFrontSpeed > 400)
	DriveLeftFrontSpeed = 400;

	if(DriveLeftRearSpeed < -400)
	DriveLeftRearSpeed = -400;
	if(DriveLeftRearSpeed > 400)
	DriveLeftRearSpeed = 400;

	prevDriveLeftFrontSpeed = torqueLimit(prevDriveLeftFrontSpeed, DriveLeftFrontSpeed, TorqueLimitDrive);

	mc2.setM1Speed(prevDriveLeftFrontSpeed);

	prevDriveLeftRearSpeed = torqueLimit(prevDriveLeftRearSpeed, DriveLeftRearSpeed, TorqueLimitDrive);

	mc2.setM2Speed(prevDriveLeftRearSpeed);

	//Serial.print(DriveLeftSpeed);
	//Serial.print("\t");


	if(LiftSpeed < -400)
	LiftSpeed = -400;
	if(LiftSpeed > 400)
	LiftSpeed = 400;

	prevLiftSpeed = torqueLimit(prevLiftSpeed, LiftSpeed, TorqueLimitLift);

	serLiftLeft.write(converToServo(prevLiftSpeed));
	serLiftRight.write(converToServo(prevLiftSpeed));

	/* Serial.print(LiftLeftSpeed);
	Serial.print("\t");
*/

	//Serial.print(LiftRightSpeed);
	//Serial.print("\t");

	//IntakeSpeed

	if(ClawSpeed < -400)
	ClawSpeed = -400;
	if(ClawSpeed > 400)
	ClawSpeed = 400;
	serClaw.write(convertToServo(ClawSpeed));


	analogWrite(_ledRed, LEDRed);
	analogWrite(_ledGrn, LEDGreen);
	analogWrite(_ledBlu, LEDBlue);


	//Expect 0-255
	/*float LEDRed;
	float LEDBlue;
	float LEDGreen;*/

}

int Robot::convertToServo(float inVal)
{

	if(inVal > 0)
	{
		//Serial.println(((inVal/400 * (_servoMax - _servoNeut)) + _servoNeut));
		return ((inVal/400 * (_servoMax - _servoNeut)) + _servoNeut);
	}
	else if(inVal < 0)
	{
		//Serial.println(((inVal/400 * (_servoNeut - _servoMin)) + _servoNeut));
		return ((inVal/400 * (_servoNeut - _servoMin)) + _servoNeut);
	}
	else
	return _servoNeut;
}


//ReadOnly Methods
/*
float Robot::GetEncDriveRight(){
	return _encDriveRight;
}

float Robot::GetEncDriveLeft(){
	return _encDriveLeft;
}

float Robot::GetEncLiftRight(){
	return _encLiftRight;
}

float Robot::GetEncLiftLeft(){
	return _encLiftLeft;
}

float Robot::GetEncClaw(){
	return _encClaw;
}

bool Robot::GetLeftLimitSwitch(){
	return _leftLimitSwitch;
}

bool Robot::GetRightLimitSwitch(){
	return _rightLimitSwitch;
}
*/

float Robot::GetDriveLeftFrontCurrent(){
	return _driveLeftFrontCurrent;
}

float Robot::GetDriveRightFrontCurrent(){
	return _driveRightFrontCurrent;
}

float Robot::GetDriveLeftRearCurrent(){
	return _driveLeftRearCurrent;
}

float Robot::GetDriveRightRearCurrent(){
	return _driveRightRearCurrent;
}

/*
void Robot::SetController(Controller *p)
{
controller = p;
}
*/
