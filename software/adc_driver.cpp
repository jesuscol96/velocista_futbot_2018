
#include "adc_driver.h"
#include "linear.h"


//----------------------------------------------------------------------------------------------------------------------------------------------
	
//Functions to handle an ADC Chip

void SPI_init(){

	//Enables SPI BUS
	SPI.begin();
	SPI.setMOSI(11);
	SPI.setMISO(12);
	SPI.setSCK(13);

}

void ADC_begin(int pin_SS){

	pinMode(pin_SS, OUTPUT);  //set SS as an output		
	digitalWriteFast(pin_SS, HIGH);			

	//Initialize ADC
	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
	//set SS low to enable SPI communication
	digitalWriteFast(pin_SS, LOW);		
	//
	SPI.transfer(ADC_RESET_FIFO);
	SPI.transfer(ADC_RESET_REG);
	SPI.transfer(ADC_AVG_MODE);
	SPI.transfer(ADC_SETUP);
	//
	digitalWriteFast(pin_SS, HIGH);
	SPI.endTransaction();

}

//Request a convertion of the 16 ports of the ADC and store results in the internal FIFO.
void ADC_Convert(int pin_SS){
	
	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
	//set SS low to enable SPI communication
	digitalWriteFast(pin_SS, LOW);		
	//	
	SPI.transfer(ADC_CONVERT);
	//			
	digitalWriteFast(pin_SS, HIGH);
	SPI.endTransaction();
}

//reads just one ADC result from FIFO.
uint8_t ADC_read(int pin_SS){

	uint8_t result;
	uint8_t none=0;

	SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
	//set SS low to enable SPI communication
	digitalWriteFast(pin_SS, LOW);		
	//	
	result=(SPI.transfer(none) & 0x0F) << 4;
	result|=((SPI.transfer(none) & 0xF0) >> 4);
	//	
	digitalWriteFast(pin_SS, HIGH);
	SPI.endTransaction();

	return result;

}

//reads all ADC results and put them in an array. 
void ADC_read_FIFO(int *values, int pin_SS){

	for (int i = 0; i < 16; i++){
		*(values+i)=ADC_read(pin_SS);
	}

}

//----------------------------------------------------------------------------------------------------------------------------------------------

//Functions for evaluation data

void get_line_parameters(int *sensor, double *m, double *b, double *R2){

	double x[49]={0}, y[49]={0}; //data vectors
	int n=0; //amount of data points

	//sensor[i][j]	
	for(int i = 0; i < 7; i++)
		for(int j = 0; j < 7; j++)			
			if(*(sensor+7*i+j)){  //Register data if sensor ON
				y[n]=1+j;
				x[n]=7-i;
				n++;
			}	

	if(n>=2){
		//Create object linear
		Linear line(n,x,y);
		//Put line parameters
		*m=line.getSlope();
		*b=line.getIntercept();
		*R2=line.getCoefficient();
	}
}









//----------------------------------------------------------------------------------------------------------------------------------------------

//Class implementation
matrix::matrix(int _pin_SS1,int _pin_SS2,int _pin_SS3,int _pin_EOC){

	pin_SS1=_pin_SS1;
	pin_SS2=_pin_SS2;
	pin_SS3=_pin_SS3;
	pin_EOC=_pin_EOC;
	pinMode(pin_EOC, INPUT); 

	SPI_init();
	ADC_begin(pin_SS1);
	ADC_begin(pin_SS2);
	ADC_begin(pin_SS3);		       

}

void matrix::eval_matrix(int *sensor){

	//Request results from ADC
	ADC_Convert(pin_SS1);
	ADC_Convert(pin_SS2);
	ADC_Convert(pin_SS3);

	//Wait convertions to be done
	while(digitalRead(pin_EOC)){}

	ADC_read_FIFO(sensor,pin_SS1);
	ADC_read_FIFO(sensor+16,pin_SS2);
	ADC_read_FIFO(sensor+32,pin_SS3); 

}

void matrix::sort(int *sensor){

	int buffer[48];

	for (int i = 0; i < 48; i++)
		buffer[i]=*(sensor+i);	

	for (int i = 0; i < 48; i++)		
		*(sensor+sort_data[i])=buffer[i];
}


void matrix::bin(int *sensor){

	for (int i = 0; i < 48; i++){

		if(*(sensor+i)>THRESHOLD)
			*(sensor+i)=1;
		else
			*(sensor+i)=0;
	}
}


