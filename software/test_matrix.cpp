/*
*
*  Este archivo permite mostrar mediante conexion serial con la PC
*  la lectura de la matriz de sensores.
*/

#include "WProgram.h"
#include "mylib.h"
#include "adc_driver.h"


int main(){

	

	Serial.begin(9600);
	int pin_EOC=14;  //End of conversion
	int pin_SS1=15;  //Slave Select 1
	int pin_SS2=16;  //Slave Select 2
	int pin_SS3=17;  //Slave Select 3
	

	while(!Serial.available()){}
	Serial.printf("START\n");

	matrix test(pin_SS1, pin_SS2, pin_SS3, pin_EOC);	
	//Array to store results.
	int sensor[49]={0};
	int sensor_bin[49]={0};

	//int n=0;
	


	while(1){

		/*  Solo para seleccionar un sensor a la vez

		if(Serial.available()){
				char a;
				scan_s(&a);
				n=atoi(&a);				
			}

			*/

		
		test.eval_matrix(sensor);
		test.sort(sensor);

		for (int i = 0; i < 48; i++)
			sensor_bin[i]=sensor[i];

		test.bin(sensor_bin);

		Serial.printf("------------------------------------\n");

		//Serial.printf("LED %d: %d\n",n, sensor[n]);	
			
		//Print results on screen.
		for (int i = 0; i < 7; i++){
			for (int j = 0; j < 7; j++)			
				Serial.printf("%d  ",*(sensor+7*i+j));
			Serial.printf("\n");
		}

		Serial.printf("------------------------------------\n");	

		for (int i = 0; i < 7; i++){
			for (int j = 0; j < 7; j++)			
				Serial.printf("%d  ",*(sensor_bin+7*i+j));
			Serial.printf("\n");
		}

		//Wait a sec before continue
		delay(50);			

	}
	

	


	return 0;


	
	
}

