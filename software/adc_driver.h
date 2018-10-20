
#ifndef ADC_DRIVER
#define ADC_DRIVER
#include "SPI.h"


//CONVERT REGISTER
#define ADC_CONVERT (uint8_t) 0b11111000
//SETUP REGISTER VALUES
#define ADC_SETUP (uint8_t) 0b01100100 
//AVERAGE REGISTER
#define ADC_AVG_MODE (uint8_t) 0b00100000
//RESET REGISTER VALUES
#define ADC_RESET_FIFO (uint8_t) 0b00011000   
#define ADC_RESET_REG (uint8_t) 0b00010000

#define THRESHOLD 110

//----------------------------------------------------------------------------------------------------------------------------------------------
//Functions for communication


void SPI_init();

//Initialize chip
void ADC_begin(int pin_SS);

//Request a convertion of the 16 ports of the ADC and store results in the internal FIFO.
void ADC_Convert(int pin_SS);

//reads just one ADC result from FIFO.
uint8_t ADC_read(int pin_SS);

//reads all ADC results and put them in an array. 
void ADC_read_FIFO(int *values, int pin_SS);

//----------------------------------------------------------------------------------------------------------------------------------------------

 //Functions for evaluating data

//Interpret data from matrix as a line.
void get_line_parameters(int *sensor, double *m, double *b, double *R2);







//----------------------------------------------------------------------------------------------------------------------------------------------

//Class implementation


class matrix{

	int pin_EOC;  //End of conversion
	int pin_SS1;  //Slave Select 1
	int pin_SS2;  //Slave Select 2
	int pin_SS3;  //Slave Select 3
	int sort_data[48]={9,2,43,36,29,22,15,8,1,42,35,28,21,14,7,0,25,18,11,4,45,38,31,24,17,10,3,44,37,30,23,16,41,34,27,20,13,6,47,40,33,26,19,12,5,46,39,32};

	public:	

	matrix(int _pin_SS1,int _pin_SS2,int _pin_SS3,int _pin_EOC);

	//Stores all sensor´s values in a array
	void eval_matrix(int *sensor);
	void sort(int *sensor);
	void bin(int *sensor);

};


#endif