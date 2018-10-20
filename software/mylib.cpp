
	#include "mylib.h"

	//freq_encoder CLASS
	freq_encoder::freq_encoder(int pin1, int pin2):Encoder(pin1, pin2){

		this->write(0);
		count=0;
		time_track=micros();		
	}

	//Returns frequency in Hz
	float freq_encoder::eval_freq(){

		time=micros()-time_track;
		int read=this->read();		
		float freq= (float) (read-count);
		freq=(freq*2500.0)/( (float) time);	
		count=read;
		time_track=micros();

		return freq;	
	}
	//return W
	float freq_encoder::eval_afreq(){

		float ang_freq= this->eval_freq();
		ang_freq*=(2.0*pi);

		return ang_freq;
	}
	//Returns rot angle in rad.
	float freq_encoder::eval_ang(){

		float angle= (float) this->read();
		angle=(angle/400.0)*2.0*pi;

		return angle;
	}

	//End of freq_encoder CLASS




	//MyPID CLASS
	myPID::myPID(double Kp_, double Ki_, double Kd_){

		this->Kp=Kp_;
		this->Ki=Ki_;
		this->Kd=Kd_;
		time_track=micros();

	}


	void myPID::eval_const(){

		int t=micros();
		double ts= (double) (t-time_track);
		ts/=1000000;  //micros to seconds
		time_track=t;

		//Values
		A=Kp+Ki*ts+(Kd/ts);		
		B=-Kp-(2*Kd/ts);
		C=Kd/ts;
	}


	double myPID::eval_PID(double input_value){

		//Calculate constants for controller.
		this->eval_const();
		

		if(n<2){

			input[1]=input[0];
			input[0]=input_value;
			n++;
		}
		else{

			//Update output
			output+=A*input_value+B*input[0]+C*input[1];

			//Remember new values.
			input[1]=input[0];
			input[0]=input_value;


		}


		return output;

	}

	//End of myPID CLASS


	//scan string function

	void scan_s(char *buf){

	int size=0;
	int i=0;

	while(size==0){
		size=Serial.available();		
	}
	
	if(size!=0){
		for(i=0;i<size;i++)
			*(buf+i)=Serial.read();
	}
	*(buf+i)='\0';		
}
