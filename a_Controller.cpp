#include "a_Controller.h"



// Constructors ////////////////////////////////////////////////////////////////
Controller::Controller()
	:Xbox(&dummyXboxPointer)
{

}

Controller::Controller(USB *p)
	: Xbox(p)
{
	LeftJoystickY = 0.0;
	LeftJoystickX = 0.0;
	RightJoystickY = 0.0;
	RightJoystickX = 0.0;
	TriggerAggregate = 0.0;
	LR2Aggregate = 0;
	DPadLeftRight = 0;
	APress = 0;
	YPress = 0;
	StartButton = 0;
}

void Controller::Task()
{
	//Set Controller Defaults
	LeftJoystickY = 0.0;
	LeftJoystickX = 0.0;
	RightJoystickY = 0.0;
	RightJoystickX = 0.0;
	TriggerAggregate = 0.0;
	LR2Aggregate = 0;
	APress = 0;
	YPress = 0;
	DPadLeftRight = 0;
	StartButton = 0;

	//This is where you update controller items.
	if (Xbox.XboxReceiverConnected)
	{
		//Set so player 1 takes priority
		for (uint8_t i = 0; i < 4; i++)
		{


			if (Xbox.Xbox360Connected[i])
			{



				if (Xbox.getAnalogHat(LeftHatY, i) > joystickMinThresh || Xbox.getAnalogHat(LeftHatY, i) < -joystickMinThresh)
				{
					LeftJoystickY = 400.0 / 32767 * Xbox.getAnalogHat(LeftHatY, i);
				}

				if (Xbox.getAnalogHat(LeftHatX, i) > joystickMinThresh || Xbox.getAnalogHat(LeftHatX, i) < -joystickMinThresh)
				{
					LeftJoystickX = 400.0 / 32767 * Xbox.getAnalogHat(LeftHatX, i);
				}


				if (Xbox.getAnalogHat(RightHatY, i) > joystickMinThresh || Xbox.getAnalogHat(RightHatY, i) < -joystickMinThresh)
				{
					RightJoystickY = 400.0 / 32767 * Xbox.getAnalogHat(RightHatY, i);
				}

				if (Xbox.getAnalogHat(RightHatX, i) > joystickMinThresh || Xbox.getAnalogHat(RightHatX, i) < -joystickMinThresh)
				{
					RightJoystickX = 400.0 / 32767 * Xbox.getAnalogHat(RightHatX, i);
				}

				//L2 Trigger
				if (Xbox.getButtonPress(R2, i))
				{
					TriggerAggregate = 400.0 / 255 * Xbox.getButtonPress(R2, i) * 1;
				}
				//R2 Trigger
				else if (Xbox.getButtonPress(L2, i))
				{
					TriggerAggregate = 400.0 / 255 * Xbox.getButtonPress(L2, i) * -1;
				}



				if (Xbox.getButtonPress(L1, i))
				{
					LR2Aggregate = 1;

				}
				//R1 Button
				else if (Xbox.getButtonPress(R1, i))
				{
					LR2Aggregate = -1;

				}



				if (Xbox.getButtonPress(Y, i))
				{
					YPress = 1;
				}

				if (Xbox.getButtonPress(A, i)) {

				}



				if (Xbox.getButtonPress(A, i))
				{
					APress = 1;
				}
				if (Xbox.getButtonClick(START, i))
				{
					StartButton = 1;
				}


				if (Xbox.getButtonClick(LEFT, i))
				{
					DPadLeftRight = -1;
				}
				else if (Xbox.getButtonPress(RIGHT, i))
				{
					DPadLeftRight = 1;
				}
				else if (Xbox.getButtonClick(UP, i))
				{
					DPadLeftRight = 2;
				}
				else if (Xbox.getButtonPress(DOWN, i))
				{
					DPadLeftRight = -2;
				}


			}
		}
	}
}