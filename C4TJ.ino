//a

//Indirectly Used Library Includes
#include "XBOXRECV.h"
#include "DualVNH5019MotorShield.h"
#include "SimpleTimer.h"
#include "PID_v1.h"
#include "Encoder.h"
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"

//Directly Used Library Includes
#include "a_Robot.h"
#include "a_Controller.h"
#include "a_Lift.h"
#include "a_Drive.h"
#include "a_Claw.h"

//try initializing this in the INO
Adafruit_BNO055 gyroLift = Adafruit_BNO055(55);

Robot Robot;


Controller Controller(&Robot.Usb);

Lift Lift(&Robot);
Drive Drive(&Robot);
Claw Claw(&Robot);

SimpleTimer timer;

bool _autoRunning = false;
int _autoProgNum = 0;
int _autoInterval = 0;
bool _pauseForPID = false;

const int _maxAutonomousDuration = 60000;

unsigned long serialTime;
/*
double Setpoint, Input, Output;
PID myPID;
*/

void setup() {
	Serial.begin(115200);
	while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection

	//gyroLift in INO 
	if (!gyroLift.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1);
	}

	delay(1000);

	gyroLift.setExtCrystalUse(true);

	Robot.init();
	Lift.init();
	Drive.init();

	//Robot.SetLED(255, 0, 0); //Set LEDs to white

	timer.setInterval(1, taskController);

	//gyro can only refresh every 10ms. Setting to 20 to be nice!
	timer.setInterval(20, refreshGyro);

	timer.setInterval(20, runStuff);

	timer.setInterval(100, SerialOutput);

	//timer.setInterval(250, blink);

}

void SerialOutput()
{
	Serial.print("G: ");
	Serial.print(Robot.GetGyroDegrees(), 4);

	Serial.print("P: ");
	Serial.print(Robot.GetPotLift(), 4);


	Serial.print("\tL: ");
	Serial.print(Robot.GetEncDriveLeft(), 4);


	Serial.print("\tR: ");
	Serial.print(Robot.GetEncDriveRight(), 4);


	Serial.println("");
}


void refreshGyro()
{
	/* Get a new sensor event */
	sensors_event_t event;
	gyroLift.getEvent(&event);

	/* Display the floating point data */

	Robot.SetGyroDegrees(event.orientation.x);



}

void taskController()
{
	Robot.TaskUSB();
}

void runStuff()
{


	Controller.Task();

	MapRobot();

	Lift.Task();
	Drive.Task();
	Claw.Task();

	Robot.Write();

	 SerialReceiveMove();

	 /*

	 Serial.print(Robot.GetEncDriveLeft());
	 Serial.print("\t");
	 Serial.print(Robot.GetEncDriveRight());
	 Serial.print("\t");
	 Serial.print(Robot.GetEncLiftLeft());
	 Serial.print("\t");
	 Serial.print(Robot.GetEncLiftRight());
	 Serial.print("\t");
	 Serial.println(Robot.GetEncClaw());

   */
   /* Serial.print("LLiftI: ");
	Serial.print(digitalRead(19));
	Serial.print("\t LLiftD: ");
	Serial.print(digitalRead(29));
	Serial.print("\t RDriveI: ");
	Serial.print(digitalRead(3));
	Serial.print("\t RDriveD: ");
	Serial.println(digitalRead(23)); */






	/*
	  if(millis()>serialTime)
	  {
		myPID = Drive.drivePID;
		Setpoint = Drive.driveSetPoint;
		Input = Drive.driveCurPos;
		Output = Drive.drivePIDOut;

		SerialReceive();
		SerialSend();
		serialTime+=500;
	  }
	*/
	if (_autoRunning)
	{
		switch (_autoProgNum)
		{
		case 1:
			autonomous();

		default:
			break;
		}
		if (!_pauseForPID)
			_autoInterval = _autoInterval + 20;

		//Safety switch in case forgot to call autoStop
		if (_autoInterval > _maxAutonomousDuration)
		{
			//autoStop();
		}
	}


	//Grab Serial Input


}

void loop() {
	//put this here to read encoders as fast as possible.
	Robot.Read();

	timer.run();



}

void MapRobot()
{
	//Serial.println(Robot.GetGyroDegrees());
	if (!_autoRunning) {
		Drive.LeftControllerSpeedY = Controller.LeftJoystickY;
		Drive.LeftControllerSpeedX = Controller.LeftJoystickX;
		Drive.RightControllerSpeedY = Controller.RightJoystickY;
		Drive.RightControllerSpeedX = Controller.RightJoystickX;

		//if(Controller.TriggerAggregate != 0){
		  //Lift.LiftTo(0);
		Lift.ControllerSpeed = Controller.TriggerAggregate;
		//}


		switch (Controller.DPadLeftRight) {
		case 1: //Right
			Lift.ControllerSpeed = 400;
			break;

		case -2: //DOWN
			Lift.UseLimits(false);
			Lift.ControllerSpeed = -400;
			break;

		case 2: //UP
			Lift.LiftTo(Lift.PotHighFence);
			break;

		case -1: //LEFT
			Lift.LiftTo(Lift.PotLowFence);
			break;

		}

		Claw.ControllerSpeed = Controller.LR2Aggregate;
	}

	else
	{
		Drive.LeftControllerSpeedY = 0;
		Drive.LeftControllerSpeedX = 0;
		Drive.RightControllerSpeedY = 0;
		Drive.RightControllerSpeedX = 0;
		Lift.ControllerSpeed = 0;
		Claw.ControllerSpeed = 0;


	}


	if (Controller.StartButton == 1)
	{
		Drive.HeadingLockToggle();
	}

	if (Controller.YPress == 1 && !_autoRunning)
	{
		_autoProgNum = 1;
		if (_autoProgNum != 0)
		{
			if (!_autoRunning)
			{
				// Start Program
				autoStart();
			}
			else
			{
				// Stop Program
				autoStop();
			}
		}
	}

}

// ===========================================
// AUTONOMOUS METHODS
// ===========================================



// Initialize Autonomous Mode
void autoStop()
{
	_autoRunning = false;
	_autoProgNum = 0;
	_autoInterval = 0;
}

void autoStart()
{
	if (_autoProgNum != 0)
	{
		_autoRunning = true;
		_autoInterval = 0;
	}
}

int curStep = 0;
bool stop = false;


void autonomous()
{
	if (!stop) {

		// Serial.println(_autoInterval);
		switch (_autoInterval)
		{

		case 0:
			Lift.LiftTo(Lift.PotHighFence);
			Drive.Move(300);
			break;

		case 500:
			Claw.Move(-400, 2);
			Drive.Move(-300);
			break;

		case 1000:
			Drive.Move(1600);
			break;
		case 3000:
			Drive.Move(-800);
			Lift.LiftTo(Lift.PotLowVal);
			break;
		case 4500:
			Drive.Turn(90);
			break;
		case 6500:
			Drive.Move(400);
			break;
		case 7500:
			Claw.Move(400, 500);
			break;
		case 9000:
			Lift.LiftTo(Lift.PotHighFence);
			break;
		case 10000:
			Drive.Turn(-20);
			break;
		case 12000:
			Drive.Move(800);
			break;
		case 13500:
			Claw.Move(-400, 1);
			break;
		


		case 15000:
			autoStop();
			break;

		}

	}
}


void SerialReceiveMove()
{




///////////////////////////////////

  String readString, funcName;
  int funcVal;

  readString="";

  while (Serial.available()) {
    delay(10);  //delay to allow buffer to fill 
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
  } 
}

if (readString.length() >0) {
      Serial.println(readString); //see what was received
      
      // expect a string like XX #### containing the two servo positions      
      funcName = readString.substring(0, 2); //get the first four characters
      String tmpFuncVal = readString.substring(3, 7); //get the next four characters 
      funcVal = tmpFuncVal.toFloat();


  } 
  Serial.flush();  

//LIFT
  if (funcName == "LT")
  {
      Lift.LiftTo(funcVal);
  }
  else if (funcName == "LP")
  {
    Lift.SetLiftKP(funcVal);
  }
  else if (funcName == "LI")
  {
    Lift.SetLiftKI(funcVal);
  }
  else if (funcName == "LD")
  {
    Lift.SetLiftKD(funcVal);
  }
  else if (funcName == "LO") //Torque Limit Lift
  {
    Lift.SetTorqueLimit(funcVal);
  }

//DRIVE
  else if (funcName == "DT")
  {
      Drive.Turn(funcVal);
  }
  else if (funcName == "DM")
  {
      Drive.Move(funcVal);
  }
  else if (funcName == "DO") //Torque Limit Drive
  {
    Drive.SetTorqueLimit(funcVal);
  }  // DP DI DD TP TI TD HP HI HD
  else if (funcName == "DP")
  {
    Drive.SetDriveKP(funcVal);
  }
  else if (funcName == "DI")
  {
    Drive.SetDriveKI(funcVal);
  }
  else if (funcName == "DD")
  {
    Drive.SetDriveKD(funcVal);
  }
  else if (funcName == "TP")
  {
    Drive.SetTurnKP(funcVal);
  }
  else if (funcName == "TI")
  {
    Drive.SetTurnKI(funcVal);
  }
  else if (funcName == "TD")
  {
    Drive.SetTurnKD(funcVal);
  }
  else if (funcName == "HP")
  {
    Drive.SetHLKP(funcVal);
  }
  else if (funcName == "HI")
  {
    Drive.SetHLKI(funcVal);
  }
  else if (funcName == "HD")
  {
    Drive.SetHLKD(funcVal);
  }

//CLAW
  else if (funcName == "CC")
  {
      Claw.Clamp();
  }
  else if (funcName == "CO")
  {
      Claw.Open();
  }

//OTHER
  else if (funcName == "AT")
  {
    autonomous();
  }


}