
#ifndef __MYLIB__
#define __MYLIB__
#define pi 3.14159

#include "WProgram.h"
#include "Encoder.h"
#include "Metro.h"


//Class used to read encoder's signals
class freq_encoder: public Encoder{
	
	int time_track,time, count;

	public:

	freq_encoder(int pin1, int pin2);

	//Returns frequency in Hz
	float eval_freq();
	//return W
	float eval_afreq();

	//Returns rot angle in rad.
	float eval_ang();

};

//CLass to implement PID controller

class myPID{

	double input[2]={0};
	double output=0;
	int time_track;
	double A, B, C;
	double Kp,Ki,Kd;
	int n=0;


public:

	myPID(double Kp_, double Ki_, double Kd_);

private:
	void eval_const();

public:
	double eval_PID(double input_value);
	
	
};

void scan_s(char *buf);


#endif



