

#include "a_Robot.h"

// Constructors ////////////////////////////////////////////////////////////////


Robot::Robot()

{
	Usb.Init();
	mc.init();

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
	LiftSpeed = 0;
	ClawSpeed = 0;

	//Expect 0-255
	LEDRed = 0;
	LEDBlue = 0;
	LEDGreen = 0;

	TorqueLimitDrive = 0;
	TorqueLimitLift = 0; 

	LiftPotVal = 0;

}

void Robot::init(){

	pinMode(_potPin, INPUT);
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
	LiftPotVal = analogRead(_potPin);




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

	DriveRightFrontSpeed = map(DriveRightFrontSpeed, -400, 400, -255, 255);

	if(DriveRightRearSpeed < -400)
	DriveRightRearSpeed = -400;
	if(DriveRightRearSpeed > 400)
	DriveRightRearSpeed = 400;

	DriveRightRearSpeed = map(DriveRightRearSpeed, -400, 400, -255, 255);

	prevDriveRightFrontSpeed = torqueLimit(prevDriveRightFrontSpeed, DriveRightFrontSpeed, TorqueLimitDrive);

	mc.setMotorSpeed(1, prevDriveRightFrontSpeed);

	prevDriveRightRearSpeed = torqueLimit(prevDriveRightRearSpeed, DriveRightRearSpeed, TorqueLimitDrive);

	mc.setMotorSpeed(0, prevDriveRightRearSpeed * -1);

	//Serial.print(DriveRightSpeed);
	//Serial.print("\t");

	//DriveLeftSpeed
	if(DriveLeftFrontSpeed < -400)
	DriveLeftFrontSpeed = -400;
	if(DriveLeftFrontSpeed > 400)
	DriveLeftFrontSpeed = 400;

	DriveLeftFrontSpeed = map(DriveLeftFrontSpeed, -400, 400, -255, 255);

	if(DriveLeftRearSpeed < -400)
	DriveLeftRearSpeed = -400;
	if(DriveLeftRearSpeed > 400)
	DriveLeftRearSpeed = 400;

	DriveLeftRearSpeed = map(DriveLeftRearSpeed, -400, 400, -255, 255);

	prevDriveLeftFrontSpeed = torqueLimit(prevDriveLeftFrontSpeed, DriveLeftFrontSpeed, TorqueLimitDrive);

	mc.setMotorSpeed(4, prevDriveLeftFrontSpeed * -1);

	prevDriveLeftRearSpeed = torqueLimit(prevDriveLeftRearSpeed, DriveLeftRearSpeed, TorqueLimitDrive);

	mc.setMotorSpeed(5, prevDriveLeftRearSpeed * -1);

	//Serial.print(DriveLeftSpeed);
	//Serial.print("\t");


	if(LiftSpeed < -400)
	LiftSpeed = -400;
	if(LiftSpeed > 400)
	LiftSpeed = 400;

	LiftSpeed = map(LiftSpeed, -400, 400, -255, 255);

	if(LiftSpeed < 0){
		mc.setMotorSpeed(3, -1 * LiftSpeed);
		mc.setMotorSpeed(7, -1 * LiftSpeed);
	}else{
		mc.setMotorSpeed(3, -1 * LiftSpeed);
		mc.setMotorSpeed(7, -1 *LiftSpeed);
	}


	//prevLiftSpeed = torqueLimit(prevLiftSpeed, LiftSpeed, TorqueLimitLift);



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
	
	ClawSpeed = map(ClawSpeed, -400, 400, -255, 255);

	mc.setMotorSpeed(2, ClawSpeed);


	mc.setMotorSpeed(6, 0);


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
