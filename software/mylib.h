
#ifndef __MYLIB__
#define __MYLIB__
#define pi 3.14159

#include "WProgram.h"
#include "Encoder.h"
#include "Metro.h"

#define ANGLE_CONTROL true
#define TURNING_CONTROL false

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
	double A, B, C;
	double Kp,Ki,Kd;
	int n=0;

public:

	myPID(double Kp_, double Ki_, double Kd_,int ts_); //ts in micros

public:
	double eval_PID(double input_value);
	
	
};

class control_motors{

	myPID* vcomun;  //PID for common voltage
	myPID* vdif;  //PID for differential voltage
	freq_encoder* enc_m1;  //Encoder at M1
	freq_encoder* enc_m2;  //Encoder  at M2
	int pin1_m1, pin2_m1, pin1_m2, pin2_m2;  //Pins for motors output. M1=left motor, M2=right motor
	double R=0.025; //wheel's radius in meters
	double k_turning=0.2;  //Wheel's freq to vehicle's freq ratio
	double Vmax=13.0;
	double res=65535.0;

public:
	control_motors(myPID* vcomun_,myPID* vdif_,freq_encoder* enc_m1_, freq_encoder* enc_m2_,
				   int pin1_m1_, int pin2_m1_,int pin1_m2_,int pin2_m2_);

	void set(double sp_vel, double m_angle, bool control_type);  //velocity in m/s and angle in rad
	  //vel is used as sp for the vel control system, angle is used as real time measure for close
	  //loop retroalimentation.

};

class data_analyzer{  //This class provides methods to analyze matrix sensors digital output

	int* sensor;
	int size=49; //array size

	//Variables for get_dir algorithm
	int n=4;
	double Kadj=(4.0/255.0);
	double wi[4]={1, 1/2, 1/4, 1/8};  //n=4
	double w_y=0.1;
	double x_r[4]={0}; //n=4
	double y_past_r=0;
	double x_l[4]={0}; //n=4
	double y_past_l=0;

public:
	data_analyzer(int *sensor_); //constructor

	double get_slope(void); //returns aprox slope

	double get_angle(void); //Returns angle

	double get_b(void);  //returns y at x=0

	double get_error(void);	//Measuare 'LMS' error related to lineal model

	double get_mean(void);  //Returns array's mean value

	int get_dir(void); //Suggest turning direction. 1 for right, 0 for left		

private:
	
	double get_half_mean(bool side); //Get mean of half array. True for right, false for left

	double sigmoid(double x); //Returns simple sigmoid function for x

	double analyze_side(bool side); //Keeps track of a side's light weight evolution. True for right, false for left
		

};

void scan_s(char *buf);


#endif



