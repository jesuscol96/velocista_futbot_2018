/*
*  Este programa permite medir la constante de tiempo
*  de un motor DC, en este caso los usados por el velocista
*
*/

#include "WProgram.h"
#include "Encoder.h"
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
	float  f=0;
	
	freq_encoder enc(pin1,pin2);
	//
	int ts=10;	//sample time
	Metro times(ts);
	bool con=false;
	int time_track;
	//
	

	while(1){

		if(Serial.available()){

			if(Serial.read()=='a'){
			con=true;			
			time_track=millis();}

		}

		if(con){


			if(times.check()){

				f=enc.eval_freq();				
				

				if(millis()-time_track>2000)
					con=false;			
				

				//Show result on screen.			
				Serial.printf("f= %d \nt=%d \n", (int) f, millis()-time_track);

			}
		}

	}

	return 0;
}
