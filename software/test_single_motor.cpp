/*
*   Este programa permite controlar un solo motor
*   sirve para hacer pruebas en el mismo
*/


#include "WProgram.h"
#include "Metro.h"
#include "mylib.h"


#ifndef pi
#define pi 3.14159
#endif



int main()
{
	Serial.begin(9600);
	//
	int pin1=33;
	int pin2=34;
	int pin3=10;
	freq_encoder enc(pin1,pin2);
	float w=0, f=0;
	//
	int ts=50;	//sample time
	Metro times(ts);
	//
	analogWriteResolution(16);
	double res=65535.0;  //max value for analog write
	double vol_max=20.0; //PWM max voltage 	
	
	//PID parameters	
	double input=0, output=0, sp=0;	
	double Kp=3.22;
	double Ki=1.61;
	double Kd=0;
	int pwm;
	//
	myPID motor(Kp,Ki,Kd);  //Motor 

	
	//setpoint
	sp=40;
	int size;



	while(1){
		

		if(times.check()){

			if(Serial.available()){
				char a;
				scan_s(&a);
				int sp_int=atoi(&a);
				sp= (double) sp_int;
			}

			f=enc.eval_freq();
			w=2.0*pi*f;

			//Controller
			input= sp- (double) f;
			output=motor.eval_PID(input);

			//Computing PWR			
			pwm= (int) (output*res/vol_max);		
			if(pwm>res) pwm=res;
			if(pwm<0) pwm=0;
			

			//Write analog pin
			analogWrite(pin3,pwm);

			//Show result on screen.			
			Serial.printf("sp=%d\nf= %d \nw=%d \npwm: %d\n",(int) sp, (int) f, (int) w, pwm);
			}

		

	}

	return 0;
}
