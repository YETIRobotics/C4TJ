#ifndef MDD10A_h
#define MDD10A_h

#include <Arduino.h>

class MDD10A{
	
public:

	MDD10A();

	void setMotorSpeed(int controller, int speed);
	void init();

	int dir = 0;


	//PinOuts

	const int _mc0_dir0 = 22;
	const int _mc0_pwm0 = 2;
	const int _mc0_dir1 = 24;
	const int _mc0_pwm1 = 3;
	const int _mc1_dir0 = 23;
	const int _mc1_pwm0 = 4;
	const int _mc1_dir1 = 25;
	const int _mc1_pwm1 = 5;
	const int _mc2_dir0 = 26;
	const int _mc2_pwm0 = 6;
	const int _mc2_dir1 = 28;
	const int _mc2_pwm1 = 44;
	const int _mc3_dir0 = 27;
	const int _mc3_pwm0 = 8;
	const int _mc3_dir1 = 29;
	const int _mc3_pwm1 = 9;




};

#endif


